// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package syncset

import (
	"context"
	"fmt"
	"log/slog"
	"maps"
	"os"
	"strconv"
	"strings"
	"sync"

	"intrinsic/storage/onprem_cas/internal/activeset"
)

// Controller orchestrates synchronization of active [SyncSet]s.
type Controller struct {
	// Configuration & Persistence
	syncsetDir string // Root filesystem directory where SyncSet JSON files are persisted
	store      *store // Disk-backed persistence layer for active and completed SyncSet generations

	// In-Memory Progress Tracking & Coordination
	mu        sync.Mutex            // Protects controller lifecycle updates and concurrent upserts
	tracking  *statusTracking       // Central memory-efficient status cache and progress tracker for active SyncSets
	activeSet *activeset.Controller // Underlying multi-object transfer controller managing active CAS transfers

	// Lifecycle & Concurrency Control
	bgCtx  context.Context    // Background context for the controller lifecycle and asynchronous tasks
	cancel context.CancelFunc // Cancels bgCtx upon controller shutdown (Close)
	wg     sync.WaitGroup     // Tracks active background goroutines (such as cleanup tasks) to ensure clean shutdown
}

// NewController creates a new [Controller] and reads active [SyncSet]s from disk.
// retentionDays specifies the age in days after which outdated completed SyncSets (and their LRO results) are deleted from disk during startup (0 means immediately).
func NewController(ctx context.Context, syncsetDir string, retentionDays int, transferClient activeset.Transfer) (*Controller, error) {
	if err := os.MkdirAll(syncsetDir, 0o700); err != nil {
		return nil, fmt.Errorf("creating syncset dir: %w", err)
	}

	ctx, cancel := context.WithCancel(ctx)

	c := &Controller{
		syncsetDir: syncsetDir,
		store:      newStore(syncsetDir, retentionDays),
		tracking:   newStatusTracking(),
		bgCtx:      ctx,
		cancel:     cancel,
	}
	c.activeSet = activeset.NewController(ctx, transferClient, c.onChangeActiveSet)

	if err := c.loadAndResume(); err != nil {
		return nil, fmt.Errorf("loading and resuming syncsets: %w", err)
	}

	return c, nil
}

// Close cancels the controller's context and waits for all sync/GC goroutines to finish.
func (c *Controller) Close() error {
	c.cancel()
	c.activeSet.Close()
	c.wg.Wait()
	return nil
}

// List returns all SyncSets.
func (c *Controller) List() []*SyncSet {
	list := c.store.List()
	return c.tracking.populateStatuses(list)
}

// SyncSet retrieves a specific generation of a SyncSet.
func (c *Controller) SyncSet(name string, gen Generation) (*SyncSet, error) {
	if err := validateName(name); err != nil {
		return nil, err
	}
	ss, err := c.store.SyncSet(name, gen)
	if err != nil {
		return nil, err
	}
	return c.tracking.populateStatus(ss), nil
}

// LatestSyncSet retrieves the latest generation of a SyncSet by name.
func (c *Controller) LatestSyncSet(name string) (*SyncSet, error) {
	if err := validateName(name); err != nil {
		return nil, err
	}
	ss, err := c.store.latestSyncSet(name)
	if err != nil {
		return nil, err
	}
	return c.tracking.populateStatus(ss), nil
}

// ActiveObjects returns the union of all object digests across the latest generation of all SyncSets.
func (c *Controller) ActiveObjects() map[string]struct{} {
	if c == nil || c.store == nil {
		return nil
	}
	return c.store.activeObjects()
}

// loadAndResume loads active syncsets from persistence storage and resumes reconciliation.
func (c *Controller) loadAndResume() error {
	loadedCount, failedCount, err := c.store.loadActive()
	if err != nil {
		return err
	}
	slog.Debug("SyncSet: Completed loading active generations on startup",
		slog.Int("loaded_count", loadedCount), slog.Int("failed_count", failedCount))

	list := c.store.List()
	latestByName := make(map[string]*SyncSet)
	for _, ss := range list {
		currentLatest := latestByName[ss.Name]
		if currentLatest == nil || ss.Generation > currentLatest.Generation {
			latestByName[ss.Name] = ss
		}
	}

	// Determine active (not done) SyncSets which
	// require up and/or download of objects.
	var latest []*SyncSet     // require up- and download
	var superseded []*SyncSet // require only upload
	isLatestGeneration := func(ss *SyncSet) bool {
		return latestByName[ss.Name] == ss
	}

	for _, ss := range list {
		if ss.State.IsDone() {
			continue
		}
		if len(ss.Objects) == 0 {
			c.store.Update(ss.Name, ss.Generation, func(s *SyncSet) {
				s.State = SyncStateDone
			})
			if err := c.store.persist(ss, false); err != nil {
				slog.Error("SyncSet: Failed to persist empty syncset state", sLogSyncSet(ss), slog.Any("error", err))
			}
			continue
		}
		if isLatestGeneration(ss) {
			latest = append(latest, ss)
		} else {
			superseded = append(superseded, ss)
		}
	}

	for _, ss := range latest {
		k := syncSetKey{name: ss.Name, generation: ss.Generation}
		c.tracking.add(k, ss.Objects)
		c.activeSet.Add(ss.Key(), ss.Objects, ss.Objects)
	}
	for _, ss := range superseded {
		k := syncSetKey{name: ss.Name, generation: ss.Generation}
		c.tracking.add(k, ss.Objects)
		c.activeSet.Add(ss.Key(), nil, ss.Objects)
	}
	return nil
}

// Upsert upserts a SyncSet.
// If the latest generation contains the same set of objects
// it returns the latest [SyncSet] with that name.
func (c *Controller) Upsert(name string, ds []Digest) (*SyncSet, error) {
	if err := validateName(name); err != nil {
		return nil, err
	}
	c.mu.Lock()
	defer c.mu.Unlock()

	// If the latest generation already has the same set of objects, return that.
	latestSS, err := c.store.latestSyncSet(name)
	if err == nil { // NO error
		if equalSet(latestSS.Objects, ds) {
			slog.Debug("SyncSet upserted with same objects, returning latest generation", sLogSyncSet(latestSS))
			return c.tracking.populateStatus(latestSS), nil
		}
	} else { // error, we have no latest generation with that name
		latestSS = nil
	}

	prevSS := latestSS // latest becomes superseded now

	newSS, err := c.store.New(name)
	if err != nil {
		return nil, fmt.Errorf("creating new generation: %w", err)
	}
	newSS.Objects = ds

	// Cancel transfers (specifically downloads) of previous generation now that new generation is persisted.
	// Note: We only cancel local downloads (`dls: prevSS.Objects`, `ups: nil`) on the superseded generation
	// because old generations do not require local files. Uploads continue, and newSS initiates transfers for its objects.
	if prevSS != nil {
		c.activeSet.Remove(prevSS.Key(), prevSS.Objects, nil)
	}

	if len(ds) == 0 {
		newSS.State = SyncStateDone
		if err := c.store.persist(newSS, false); err != nil {
			return nil, fmt.Errorf("persisting new generation: %w", err)
		}
		slog.Debug("SyncSet upserted with 0 objects, transitioning to DONE immediately", sLogSyncSet(newSS))
		return newSS, nil
	}

	if err := c.store.persist(newSS, false); err != nil {
		return nil, fmt.Errorf("persisting new generation: %w", err)
	}

	slog.Debug("SyncSet upserted", sLogSyncSet(newSS), slog.Int("total_objects", len(ds)))

	// Reconcile (download and upload) objects from the now active generation
	k := syncSetKey{name: newSS.Name, generation: newSS.Generation}
	c.tracking.add(k, ds)
	c.activeSet.Add(newSS.Key(), ds, ds)
	// Populate the syncset with the current object status and return
	return c.tracking.populateStatus(newSS), nil
}

// parse <name>/<generation> to <name>, <generation>, nil
func parseKey(key string) (string, Generation, error) {
	parts := strings.SplitN(key, "/", 2)
	if len(parts) != 2 {
		return "", 0, fmt.Errorf("invalid key format: %q", key)
	}
	gen, err := strconv.ParseUint(parts[1], 10, 64)
	if err != nil {
		return "", 0, fmt.Errorf("invalid generation in key %q: %v", key, err)
	}
	return parts[0], Generation(gen), nil
}

// onChangeActiveSet evaluates and updates the lifecycle state of a SyncSet when an object status changes.
func (c *Controller) onChangeActiveSet(key string, status *ObjectStatus) {
	if status == nil {
		return
	}
	// 1. Retrieve the SyncSet by key
	name, gen, err := parseKey(key)
	if err != nil {
		slog.Error("SyncSet: Invalid key in onChangeActiveSet", slog.String("key", key), slog.Any("error", err))
		return
	}
	ss, err := c.store.SyncSet(name, gen)
	if err != nil || ss == nil {
		return
	}

	// 2. Update status tracking with the new object status
	k := syncSetKey{name: name, generation: gen}
	c.tracking.updateStatus(k, status)

	// Determine if SyncSet is superseded. Superseded SyncSets do not
	// care about missing local files.
	latest, err := c.store.latestSyncSet(name)
	isSuperseded := err == nil && latest != nil && latest.Generation > gen
	newState := c.tracking.getSyncState(k, isSuperseded)

	// Update state of SyncSet in memory if it changed.
	changed := false
	c.store.Update(ss.Name, ss.Generation, func(ss *SyncSet) {
		if ss.State != newState {
			ss.State = newState
			changed = true
		}
	})

	// If the state is the same as before, return early.
	if !changed {
		return
	}

	// Otherwise persist if the SyncSet is done now.
	if newState.IsDone() {
		switch newState {
		case SyncStateDone:
			slog.Debug("SyncSet sync loop finished successfully", sLogSyncSet(ss))
		case SyncStateDoneError:
			slog.Warn("SyncSet sync loop finished with errors", sLogSyncSet(ss))
		default:
			slog.Warn("Unhandled SyncSet done state", sLogSyncSet(ss), "newState", newState)
		}
		// for failed SyncSets we want to persist the errors on disk for future debugging
		writeErrorStatus := newState == SyncStateDoneError
		if writeErrorStatus {
			c.tracking.populateStatus(ss)
		}
		// persist
		if err := c.store.persist(ss, writeErrorStatus); err != nil {
			slog.Error("Failed to persist final state", sLogSyncSet(ss), slog.Any("error", err))
		}
		// cleanup activeset object tracking
		c.wg.Go(func() {
			c.activeSet.Clear(ss.Key())
			c.tracking.delete(k)
		})
	}
}

func equalSet(a, b []string) bool {
	setA := make(map[string]struct{}, len(a))
	for _, x := range a {
		setA[x] = struct{}{}
	}
	setB := make(map[string]struct{}, len(b))
	for _, x := range b {
		setB[x] = struct{}{}
	}
	return maps.Equal(setA, setB)
}

func sLogSyncSet(ss *SyncSet) slog.Attr {
	return slog.Group("syncset",
		slog.String("name", ss.Name), slog.Uint64("generation", uint64(ss.Generation)),
		slog.String("state", string(ss.State)))
}
