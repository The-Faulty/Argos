$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$launchScript = Join-Path $scriptRoot "full_body_pose_dashboard\launch.ps1"

if (-not (Test-Path $launchScript)) {
  throw "Launch script not found: $launchScript"
}

& $launchScript @args
