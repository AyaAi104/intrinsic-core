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

// Package artifacts represents an implementation of artifact management service
package main

import (
	"context"
	"flag"
	"fmt"
	"net"
	"os"
	"os/signal"
	"time"

	"errors"
	"net/http"

	"intrinsic/kubernetes/intrinsic"
	"intrinsic/stats/go/telemetry"
	"intrinsic/storage/artifacts/artifacts"
	"intrinsic/storage/artifacts/internal/internal"

	"github.com/containerd/containerd"
	log "github.com/golang/glog"
	"go.opencensus.io/plugin/ocgrpc"
	"go.opencensus.io/plugin/ochttp"
	"google.golang.org/grpc"
	"google.golang.org/grpc/reflection"

	artifactgrpcpb "intrinsic/storage/artifacts/proto/v1/artifact_go_proto"
)

var (
	opts = artifacts.DefaultOptions()

	// containerd backed parameters
	containerdAddress   = flag.String("containerd_address", opts.Address, "Address used to connect to local containerd instance")
	containerdNamespace = flag.String("containerd_namespace", opts.Namespace, "Containerd namespace to be manipulated")
	fileStoreRoot       = flag.String("file_store_dir", opts.ObjectStoreRoot, "Local disk storage directory")
	registryPort        = flag.Int("registry_port", 0, "Port to serve local OCI distribution registry on (0 to disable)")

	// container registry flags, if --registry is set, it takes precedence over --containerd_address
	registryAddress = flag.String("registry", "", "Address of remote container registry to write images to, e.g.: gcr.io/giza-workcells")
	anonymousAccess = flag.Bool("anonymous_registry", false, "Indicates that registry does not require authorization")
	remoteUsername  = flag.String("registry_username", "", "Username to use for remote registry. If not set, workload identity is used")
	remotePassword  = flag.String("registry_password", "", "Password to use for remote registry.")

	// internal service parameters
	maxBlobSize   = flag.Int("max_blob_size", opts.MaxBlobSize, "Maximum upload size of the blob in bytes. Default 5MB")
	maxObjectSize = flag.Int64("max_object_size", opts.MaxObjectSize, "Maximum size of a single ephemeral object. Default 512MB")
	servicePort   = flag.Int("port", 0, "Service port to listen on")
	serviceAddr   = flag.String("addr", "0.0.0.0", "Address to bind service to. Defaults to all interfaces")

	// Tracing flags.
	traceProbability  = flag.Float64("trace_probability", 0.0, "The fraction of requests to upload to Stackdriver Trace.")
	prometheusPort    = flag.Int64("opencensus_metrics_port", 9101, "Which port to serve the prometheus endpoint on.")
	opencensusTracing = flag.Bool("opencensus_tracing", false, "Whether to send traces to opencensus.")
)

func startServer(ctx context.Context, addr string, port int, opts *artifacts.ServiceOptions) error {
	lis, err := net.Listen("tcp", fmt.Sprintf("%s:%d", addr, port))
	if err != nil {
		log.ErrorContextf(ctx, "serverImpl bind failed: %v", err)
	}

	svc, err := artifacts.New(ctx, opts)
	if err != nil {
		return fmt.Errorf("cannot create service: %w", err)
	}

	grpcServer := grpc.NewServer(grpc.StatsHandler(&ocgrpc.ServerHandler{}), grpc.MaxRecvMsgSize(opts.MaxBlobSize*2))
	artifactgrpcpb.RegisterArtifactServiceApiServer(grpcServer, svc)
	reflection.Register(grpcServer)
	log.InfoContextf(ctx, "Starting artifact service at %q", lis.Addr())

	go func() {
		<-ctx.Done()
		if grpcServer != nil {
			log.Info("Stopping artifact service")
			grpcServer.GracefulStop()
		}
	}()

	return grpcServer.Serve(lis)
}

// startRegistryServer initializes and starts an in-process HTTP server implementing
// the OCI Distribution Spec v1.0.1 (https://github.com/opencontainers/distribution-spec/blob/v1.0.1/spec.md)
// backed by containerd's content store and image service.
func startRegistryServer(ctx context.Context, addr string, port int, containerdAddr, namespace string) {
	if port <= 0 {
		return
	}
	client, err := containerd.New(containerdAddr, containerd.WithDefaultNamespace(namespace))
	if err != nil {
		log.WarningContextf(ctx, "cannot create containerd client for OCI registry: %v", err)
		return
	}
	lis, err := net.Listen("tcp", fmt.Sprintf("%s:%d", addr, port))
	if err != nil {
		log.WarningContextf(ctx, "cannot start OCI registry listener: %v", err)
		return
	}
	handler := internal.NewOCIRegistryHandler(client, namespace)
	server := &http.Server{
		Handler: handler,
		// Bound header reads without timing out large image uploads.
		ReadHeaderTimeout: 10 * time.Second,
	}
	go func() {
		<-ctx.Done()
		server.Shutdown(context.Background())
		client.Close()
	}()
	go func() {
		log.InfoContextf(ctx, "Starting local OCI distribution registry at %q", lis.Addr())
		if err := server.Serve(lis); err != nil && !errors.Is(err, http.ErrServerClosed) {
			log.ErrorContextf(ctx, "OCI registry server error: %v", err)
		}
	}()
}

func main() {
	intrinsic.Init()
	writeToRemoteRegistry := *registryAddress != ""
	// Set up and serve telemetry.
	go func() {
		views := append(ocgrpc.DefaultClientViews, ocgrpc.DefaultServerViews...)
		if writeToRemoteRegistry {
			// we are going to make http calls to remote repository as its client
			views = append(views, ochttp.ClientCompletedCount,
				ochttp.ClientSentBytesDistribution,
				ochttp.ClientReceivedBytesDistribution,
				ochttp.ClientRoundtripLatencyDistribution,
			)
		}
		// The returned object could be used to implement a graceful shutdown.
		telemetry.Initialize(
			telemetry.WithTracing(*opencensusTracing), // on-prem!
			telemetry.WithProbability(*traceProbability),
			telemetry.EnableMetrics(*prometheusPort),
			telemetry.WithViews(views),
		)
	}()

	ctx, cancelFx := signal.NotifyContext(context.Background(), os.Interrupt)
	defer cancelFx() // just regular cleanup

	var opts *artifacts.ServiceOptions

	if writeToRemoteRegistry {
		// setting up remote registry backend
		opts = &artifacts.ServiceOptions{
			RegistryBackend: true,
			Address:         *registryAddress,
			AnonymousAccess: *anonymousAccess,
			Username:        *remoteUsername,
			Password:        *remotePassword,
			MaxBlobSize:     *maxBlobSize,
			MaxObjectSize:   *maxObjectSize,
		}
	} else {
		// setting up containerd as backend
		opts = &artifacts.ServiceOptions{
			Address:         *containerdAddress,
			Namespace:       *containerdNamespace,
			MaxBlobSize:     *maxBlobSize,
			MaxObjectSize:   *maxObjectSize,
			ObjectStoreRoot: *fileStoreRoot,
		}
		if *registryPort > 0 {
			startRegistryServer(ctx, *serviceAddr, *registryPort, *containerdAddress, *containerdNamespace)
		}
	}

	log.InfoContextf(ctx, "starting server with following options: %#v", opts)

	if err := startServer(ctx, *serviceAddr, *servicePort, opts); err != nil {
		log.ExitContextf(ctx, "Terminating with error: %v", err)
	}
	log.InfoContext(ctx, "Done.")
}
