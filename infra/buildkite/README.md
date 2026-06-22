# Buildkite Terraform

This directory provisions Buildkite CI for OpenMowerNext without using `bk auth`.

## Prerequisites

- Terraform `>= 1.5` or OpenTofu.
- A Buildkite API token with GraphQL enabled and these scopes: `read_pipelines`, `write_pipelines`, `write_suites`, `read_clusters`, `write_clusters`, `read_teams`, `read_organizations`.
- The Buildkite GitHub App connected to `jkaflik/OpenMowerNext` if `create_webhook = true`.

## Apply

```bash
cd infra/buildkite
cp terraform.tfvars.example terraform.tfvars
export BUILDKITE_API_TOKEN="..."
terraform init
terraform validate
terraform plan
terraform apply
```

If hosted queue capacity is limited on the Buildkite plan, override the shapes in `terraform.tfvars`:

```hcl
ci_instance_shape     = "LINUX_AMD64_2X4"
webots_instance_shape = "LINUX_AMD64_2X4"
```

If the GitHub App is not connected yet, set:

```hcl
create_webhook = false
```

## State

Terraform state can contain sensitive values if secrets are added later. Keep state out of git and move it to an encrypted remote backend before managing Buildkite secrets such as GHCR tokens.

The current configuration does not create Buildkite secrets and does not push Docker images.
