data "buildkite_teams" "current" {}

locals {
  default_team_ids = [
    for team in data.buildkite_teams.current.teams : team.id
    if team.is_default_team
  ]

  default_team_id = var.default_team_id != "" ? var.default_team_id : try(local.default_team_ids[0], null)

  upload_steps = <<-YAML
    steps:
      - label: "Pipeline upload"
        command: "buildkite-agent pipeline upload"
        agents:
          queue: "${buildkite_cluster_queue.ci.key}"
  YAML
}

resource "buildkite_cluster" "openmowernext" {
  name        = var.cluster_name
  description = "Hosted CI for OpenMowerNext."
  emoji       = ":seedling:"
  color       = "#5f8f3a"
}

resource "buildkite_cluster_queue" "ci" {
  cluster_id  = buildkite_cluster.openmowernext.id
  key         = var.ci_queue_key
  description = "General OpenMowerNext CI jobs."

  hosted_agents = {
    instance_shape = var.ci_instance_shape
  }
}

resource "buildkite_cluster_queue" "webots" {
  cluster_id  = buildkite_cluster.openmowernext.id
  key         = var.webots_queue_key
  description = "OpenMowerNext Webots integration jobs."

  hosted_agents = {
    instance_shape = var.webots_instance_shape
  }
}

resource "buildkite_pipeline" "openmowernext" {
  name            = var.pipeline_name
  slug            = var.pipeline_slug
  description     = "Build, test, and validate OpenMowerNext."
  repository      = var.repository
  cluster_id      = buildkite_cluster.openmowernext.id
  default_team_id = local.default_team_id
  default_branch  = "main"
  steps           = local.upload_steps
  tags            = ["ros2", "webots", "openmowernext"]

  cancel_intermediate_builds               = true
  cancel_intermediate_builds_branch_filter = "*"

  provider_settings = {
    trigger_mode = "code"

    build_branches                         = true
    build_pull_requests                    = true
    build_pull_request_base_branch_changed = true
    build_pull_request_labels_changed      = true
    build_pull_request_ready_for_review    = true
    build_pull_request_merge_commits       = true
    build_tags                             = false

    publish_commit_status          = true
    publish_commit_status_per_step = true
    use_step_key_as_commit_status  = true

    filter_enabled = true
    filter_condition = "build.branch == \"main\" || build.pull_request.base_branch == \"main\" || build.source == \"ui\" || build.source == \"api\" || build.source == \"schedule\""
  }
}

resource "buildkite_pipeline_webhook" "github" {
  count = var.create_webhook ? 1 : 0

  pipeline_id = buildkite_pipeline.openmowernext.id
  repository  = buildkite_pipeline.openmowernext.repository
}
