terraform {
  required_version = ">= 1.5.0"

  required_providers {
    buildkite = {
      source  = "buildkite/buildkite"
      version = "~> 1.16"
    }
  }
}

provider "buildkite" {
  organization = var.organization_slug
}
