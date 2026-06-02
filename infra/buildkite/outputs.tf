output "pipeline_url" {
  description = "Buildkite pipeline URL."
  value       = "https://buildkite.com/${var.organization_slug}/${buildkite_pipeline.openmowernext.slug}"
}

output "cluster_uuid" {
  description = "Buildkite cluster UUID."
  value       = buildkite_cluster.openmowernext.uuid
}

output "ci_queue_key" {
  description = "General CI hosted queue key."
  value       = buildkite_cluster_queue.ci.key
}

output "webots_queue_key" {
  description = "Webots integration hosted queue key."
  value       = buildkite_cluster_queue.webots.key
}

output "webhook_url" {
  description = "Buildkite webhook URL if the webhook resource was created."
  value       = try(buildkite_pipeline_webhook.github[0].webhook_url, null)
}
