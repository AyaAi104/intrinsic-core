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

package vm

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"os/exec"
	"os/user"
	"strings"
	"text/tabwriter"
	"time"

	"intrinsic/kubernetes/acl/schema"
	"intrinsic/tools/inctl/cmd/root"
	"intrinsic/tools/inctl/util/cobrautil"
	"intrinsic/tools/inctl/util/color"
	"intrinsic/tools/inctl/util/printer"
	"intrinsic/tools/inctl/util/vmalias"
	"intrinsic/tools/internal/k8sclientutil"

	log "github.com/golang/glog"
	"github.com/spf13/cobra"
	"go.opencensus.io/trace"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"

	useraclpb "intrinsic/kubernetes/acl/service/api/user_acl_service_go_proto"
	leaseapigrpcpb "intrinsic/kubernetes/vmpool/manager/api/v1/lease_api_go_proto"
	leasepb "intrinsic/kubernetes/vmpool/manager/api/v1/lease_api_go_proto"
)

var debugCmd = cobrautil.ParentOfNestedSubcommands("debug", "Internal debug commands")

func init() {
	vmSetContextCmd.PersistentFlags().StringVarP(&flagContextAlias, "context-alias", "a", "", "Instead of the VM name use an alias for the kubectl context (e.g., minikube).")
	debugCmd.AddCommand(vmSetContextCmd)
	debugCmd.AddCommand(vmCleanupContextsCmd)
	debugCmd.AddCommand(vmInfoCmd)
	debugCmd.AddCommand(vmMineCmd)

	vmCmd.PersistentFlags().StringVarP(&flagUserEmail, "email", "e", "", "User email to be used for access control. Optional, will fall back to $USER@google.com if unset.")

	vmCmd.AddCommand(debugCmd)
}

func getKubernetesContexts(ctx context.Context) ([]string, error) {
	cmdArgs := []string{"kubectl", "config", "get-contexts", "--no-headers"}
	cmd := exec.Command(cmdArgs[0], cmdArgs[1:]...)
	out, err := cmd.Output()
	if err != nil {
		return nil, fmt.Errorf("execute %v: %v", cmdArgs, err)
	}
	var contexts []string
	for idx, line := range strings.Split(string(out), "\n") {
		fields := strings.Fields(line)
		if len(fields) != 4 {
			log.WarningContextf(ctx, "got %d fields in line %d (0-indexed), want 4", len(fields), idx)
			continue
		}
		cluster := strings.TrimSpace(fields[1])
		context := strings.TrimSpace(fields[0])
		if strings.HasPrefix(cluster, "vmp-") {
			contexts = append(contexts, context)
		}
	}
	return contexts, nil
}

func deleteContext(ctx context.Context, context string) error {
	cmdArgs := []string{"kubectl", "config", "delete-context", "--", context}
	cmd := exec.Command(cmdArgs[0], cmdArgs[1:]...)
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("execute %v: %v", cmdArgs, err)
	}
	return nil
}

var vmSetContextCmd = &cobra.Command{
	Use: "set-context", Short: "[Intrinsic Internal] Helper to set Kubernetes context.",
	Long: "Helper to set Kubernetes context.",
	Args: cobra.ExactArgs(1),
	RunE: func(cmd *cobra.Command, args []string) error {
		vm := args[0]
		ctx := cmd.Context()
		ctx, span := trace.StartSpan(ctx, "inctl.vm.set-context")
		defer span.End()
		return SetContext(ctx, vm, flagContextAlias, vmCmdFlags.GetFlagProject())
	},
}

// SetContext sets the Kubernetes context for the given VM.
func SetContext(ctx context.Context, vmArg, contextAlias, project string) error {
	return k8sclientutil.SetContext(ctx, contextAlias, vmArg, project)
}

var vmCleanupContextsCmd = &cobra.Command{
	Use:   "cleanup-contexts",
	Short: "[Intrinsic Internal] Remove all kubectl contexts starting with 'vmp-', including aliases.",
	Long:  "Remove all kubectl contexts starting with 'vmp-', including aliases.",
	// Overriding PersistentPreRun addresses the "Error: expected --org=<org>" error.
	PersistentPreRun: func(cmd *cobra.Command, args []string) {},
	RunE: func(cmd *cobra.Command, args []string) error {
		ctx, span := trace.StartSpan(cmd.Context(), "inctl.vm.cleanup-contexts")
		defer span.End()
		return CleanupContexts(ctx)
	},
}

// CleanupContexts removes all kubectl contexts starting with 'vmp-', including aliases.
func CleanupContexts(ctx context.Context) error {
	ls, err := getKubernetesContexts(ctx)
	if err != nil {
		return fmt.Errorf("failed to get contexts: %v", err)
	}
	if len(ls) == 0 {
		fmt.Println("no vmp- clusters found")
		return nil
	}
	for _, l := range ls {
		fmt.Println("deleting context", l)
		if err := deleteContext(ctx, l); err != nil {
			fmt.Printf("failed to delete context %q: %v", l, err)
		}
	}
	return nil
}

// leaseInfos is a slice of leaseInfo with ability to be printed by the printer.Printer.
type leaseInfos struct {
	LeaseInfos []leaseInfo `json:"leaseInfos"`
	Email      string      `json:"email"`
	Org        string      `json:"org"`
}

type leaseInfo struct {
	VM      string
	Pool    string
	Expires time.Time
}

// String returns a tabulated representation of leaseInfos.
func (li leaseInfos) String() string {
	b := new(bytes.Buffer)
	w := tabwriter.NewWriter(b,
		/*minwidth=*/ 1 /*tabwidth=*/, 1 /*padding=*/, 1 /*padchar=*/, ' ' /*flags=*/, 0)
	fmt.Fprintf(w, "%s\t%s\t%s\t%s\t%s\t%s\t%s\n", "Row", "VM", "Pool", "Expires", "Remaining", "Org", "Email")
	for i, leaseInfo := range li.LeaseInfos {
		fmt.Fprintf(w, "%d\t%s\t%s\t%s\t%s\t%s\t%s\n",
			i,
			leaseInfo.VM,
			leaseInfo.Pool,
			leaseInfo.Expires.Format(time.RFC3339),
			time.Until(leaseInfo.Expires).Round(time.Second),
			li.Org,
			li.Email,
		)
	}
	w.Flush()

	// Remove the trailing newline as the pretty-printer wrapper will add one.
	return strings.TrimSuffix(b.String(), "\n")
}

// MarshalJSON converts a ListClusterDescriptionsResponse to a byte slice.
func (li leaseInfos) MarshalJSON() ([]byte, error) {
	type leaseInfoJSON struct {
		VM      string `json:"vm"`
		Pool    string `json:"pool"`
		Expires string `json:"expires"`
		Org     string `json:"org"`
		Email   string `json:"email"`
	}
	leaseInfosJSON := make([]leaseInfoJSON, len(li.LeaseInfos))
	for i, c := range li.LeaseInfos {
		leaseInfosJSON[i] = leaseInfoJSON{
			VM:      c.VM,
			Pool:    c.Pool,
			Expires: c.Expires.Format(time.RFC3339),
			Org:     li.Org,
			Email:   li.Email,
		}
	}
	return json.Marshal(struct {
		LeaseInfos []leaseInfoJSON `json:"leaseInfos"`
	}{LeaseInfos: leaseInfosJSON})
}

var vmMineCmd = &cobra.Command{
	Use:   "mine",
	Short: "[Intrinsic Internal] List the pool VMs owned by you.",
	Long: `
	List the pool VMs owned by you. Use --email to override auto-detected user.

	Be aware: the result is based on ACL rules and might not be 100% accurate.
	`,
	RunE: func(cmd *cobra.Command, args []string) error {
		ctx, span := trace.StartSpan(cmd.Context(), "inctl.vm.mine")
		defer span.End()
		conn, err := newConn(ctx)
		if err != nil {
			return err
		}
		cl, err := newLeaseClient(ctx)
		if err != nil {
			return err
		}

		return ListMyVMs(ctx, conn, cl, flagUserEmail)
	},
}

// ListMyVMs lists the VMs owned by the current user.
func ListMyVMs(ctx context.Context, conn grpc.ClientConnInterface, cl leaseapigrpcpb.VMPoolLeaseServiceClient, userEmail string) error {
	prtr, err := printer.NewPrinter(root.FlagOutput)
	if err != nil {
		return err
	}
	email, err := getUserEmail(os.Stdout, userEmail)
	if err != nil {
		return err
	}
	// deprecated owner relationship
	// TODO(b/300597714) Remove deprecated relationships
	// Should be replaced by looking for an owner role binding but this might result in
	// more false-positives.
	ucl := useraclpb.NewUserACLServiceClient(conn)
	req := &useraclpb.LookupRequest{
		ResourceType: schema.ObjCluster,
		Permission:   schema.RelOwner,
	}
	stream, err := ucl.Lookup(ctx, req)
	if err != nil {
		return fmt.Errorf("lookup failed: %w", err)
	}

	leaseInfos := leaseInfos{Email: email, Org: vmCmdFlags.GetFlagOrganization(), LeaseInfos: []leaseInfo{}}
	for {
		resp, err := stream.Recv()
		if err == io.EOF {
			break
		}
		if err != nil {
			return fmt.Errorf("lookup stream error: %w", err)
		}
		for _, rid := range resp.GetResources() {
			// filter out non-pool relations
			if !strings.HasPrefix(rid, "vmp-") {
				continue
			}
			leaseInfo := leaseInfo{VM: rid}
			lresp, err := cl.GetLease(ctx, &leasepb.GetLeaseRequest{Instance: rid})
			if status.Code(err) == codes.NotFound { // could be that the lease got expired but acls are not updated yet
				continue
			}
			if err != nil {
				return fmt.Errorf("failed to get lease: %w", err)
			}
			l := lresp.GetLease()
			leaseInfo.Pool = l.GetPool()
			leaseInfo.Expires = l.GetExpires().AsTime()
			leaseInfos.LeaseInfos = append(leaseInfos.LeaseInfos, leaseInfo)
		}
	}
	prtr.Print(leaseInfos)
	return nil
}

func getUserEmail(stdout io.Writer, userEmail string) (string, error) {
	if userEmail != "" {
		return userEmail, nil
	}
	u, err := user.Current()
	if err != nil {
		return "", fmt.Errorf("failed to get current user: %w", err)
	}
	e := fmt.Sprintf("%s@google.com", u.Username)
	return e, nil
}

var infoDesc = `
Retrieve basic information about a VM.

Current information available:
  - VM name
  - Project the VM is in

Example:
	inctl vm debug info vmp-3f30-x9t7q72u --org intrinsic-dev

You can also pass a context-alias for a VM:
	inctl vm debug info vmkube --org intrinsic-dev
`

var vmInfoCmd = &cobra.Command{
	Use:   "info",
	Short: "[Intrinsic Internal] Retrieve information about a VM.",
	Long:  infoDesc,
	Args:  cobra.ExactArgs(1),
	RunE: func(cmd *cobra.Command, args []string) error {
		ctx, span := trace.StartSpan(cmd.Context(), "inctl.vm.info")
		span.AddAttributes(trace.StringAttribute("org", vmCmdFlags.GetFlagOrganization()))
		defer span.End()
		cl, err := newLeaseClient(ctx)
		if err != nil {
			return err
		}

		return GetVMInfo(ctx, cl, args, vmCmdFlags.GetFlagProject())
	},
}

// GetVMInfo retrieves basic information about a VM.
func GetVMInfo(ctx context.Context, cl leaseapigrpcpb.VMPoolLeaseServiceClient, args []string, projectName string) error {
	rr := vmalias.ResolveResult{ // default to non-resolved values
		VM:      args[0],
		Alias:   args[0],
		Project: projectName,
	}
	if !vmalias.IsPoolVM(args[0]) { // alias given, try to resolve
		var err error
		rr, err = vmalias.Resolve(args[0])
		if err != nil {
			return fmt.Errorf("could not resolve alias: %w", err)
		}
	}
	span := trace.FromContext(ctx)
	span.AddAttributes(trace.StringAttribute("vm", rr.VM))
	if rr.Project != projectName {
		color.C.Yellow().Printf(" (you provided %q)", projectName)
	}

	// print info
	fmt.Printf("VM:\t\t%s\n", rr.VM)
	if rr.Alias != rr.VM {
		fmt.Printf("Alias:\t\t%s\n", rr.Alias)
	}
	fmt.Printf("Project:\t%s", rr.Project)
	fmt.Println()
	lresp, err := cl.GetLease(ctx, &leasepb.GetLeaseRequest{Instance: rr.VM})
	if err != nil {
		fmt.Println()
		return err
	}
	l := lresp.GetLease()
	expires := l.GetExpires().AsTime()
	fmt.Printf("Pool:\t\t%s\n", l.GetPool())
	fmt.Printf("Expires:\t%s (in %v)\n", expires.Format(time.RFC3339), time.Until(expires).Round(time.Second))
	fmt.Println()
	return nil
}
