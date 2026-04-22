$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$launchScript = Join-Path $scriptRoot "robot_dog_debug_dashboard\raspberry_pi_controller\launch_pi_controller.sh"

if (-not (Test-Path $launchScript)) {
  throw "Pi launch script not found: $launchScript"
}

Write-Host "Pi launcher script is intended to run on the Raspberry Pi:"
Write-Host "bash '$launchScript'"
