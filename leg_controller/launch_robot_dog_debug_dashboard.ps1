$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$launchScript = Join-Path $scriptRoot "robot_dog_debug_dashboard\\launch.ps1"

if (-not (Test-Path $launchScript)) {
  throw "Launch script not found: $launchScript"
}

& $launchScript
