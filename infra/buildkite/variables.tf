variable "organization_slug" {
  description = "Buildkite organization slug."
  type        = string
  default     = "kuba-kaflik"
}

variable "repository" {
  description = "Git repository URL used by the Buildkite pipeline."
  type        = string
  default     = "https://github.com/jkaflik/OpenMowerNext.git"
}

variable "cluster_name" {
  description = "Dedicated Buildkite cluster name. Buildkite cluster names cannot contain spaces."
  type        = string
  default     = "OpenMowerNext"
}

variable "pipeline_name" {
  description = "Buildkite pipeline display name."
  type        = string
  default     = "OpenMowerNext"
}

variable "pipeline_slug" {
  description = "Buildkite pipeline URL slug."
  type        = string
  default     = "openmowernext"
}

variable "default_team_id" {
  description = "Optional Buildkite team GraphQL ID for initial pipeline access. Leave empty to use the organization default team."
  type        = string
  default     = ""
}

variable "create_webhook" {
  description = "Create a GitHub App webhook for the pipeline. Requires the Buildkite GitHub App to be connected to the repository."
  type        = bool
  default     = true
}

variable "ci_queue_key" {
  description = "Buildkite hosted queue key for normal CI jobs."
  type        = string
  default     = "openmowernext-ci"
}

variable "webots_queue_key" {
  description = "Buildkite hosted queue key for Webots integration jobs."
  type        = string
  default     = "openmowernext-webots"
}

variable "ci_instance_shape" {
  description = "Buildkite hosted Linux instance shape for normal CI jobs. Override to LINUX_AMD64_2X4 on Personal plans if needed."
  type        = string
  default     = "LINUX_AMD64_4X16"
}

variable "webots_instance_shape" {
  description = "Buildkite hosted Linux instance shape for Webots integration jobs. Override to LINUX_AMD64_2X4 on Personal plans if needed."
  type        = string
  default     = "LINUX_AMD64_8X32"
}
