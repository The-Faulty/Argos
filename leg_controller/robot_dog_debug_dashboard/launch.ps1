$dashboardRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$npm = (Get-Command npm.cmd -ErrorAction Stop).Source
$pwshCommand = Get-Command pwsh -ErrorAction SilentlyContinue
$pwsh = $null
$controllerRoot = Join-Path $dashboardRoot "raspberry_pi_controller"
$controllerPort = 8787
$vitePort = 5173

if ($pwshCommand) {
  $pwsh = $pwshCommand.Source
}

if (-not $pwsh) {
  $pwsh = (Get-Command powershell.exe -ErrorAction Stop).Source
}

function Test-PortListening {
  param(
    [int]$Port
  )

  $connection = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue | Select-Object -First 1
  return $null -ne $connection
}

Set-Location $dashboardRoot

if (-not (Test-Path (Join-Path $dashboardRoot "node_modules"))) {
  & $npm install
  if ($LASTEXITCODE -ne 0) {
    throw "npm install failed."
  }
}

if (-not (Test-Path (Join-Path $controllerRoot "node_modules"))) {
  Set-Location $controllerRoot
  & $npm install
  if ($LASTEXITCODE -ne 0) {
    throw "Pi controller npm install failed."
  }
  Set-Location $dashboardRoot
}

if (Test-PortListening -Port $controllerPort) {
  Write-Host "Pi controller already appears to be running on port $controllerPort."
} else {
  $controllerCommand = "Set-Location '$controllerRoot'; & '$npm' start"
  Start-Process -FilePath $pwsh -ArgumentList "-NoExit", "-Command", $controllerCommand | Out-Null
  Write-Host "Pi controller started in a new PowerShell window."
}

if (Test-PortListening -Port $vitePort) {
  Write-Host "Dashboard dev server already appears to be running on port $vitePort."
  Write-Host "Open http://localhost:$vitePort in your browser."
  return
}

Write-Host "Starting Vite dev server in this window..."

& $npm run dev
