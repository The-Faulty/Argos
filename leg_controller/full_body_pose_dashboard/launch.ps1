param(
  [int]$BridgePort = 8787,
  [int]$VitePort = 5174
)

$ErrorActionPreference = "Stop"
$poseDashboardRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$legControllerRoot = Split-Path -Parent $poseDashboardRoot
$bridgeRoot = Join-Path $legControllerRoot "robot_dog_debug_dashboard"
$npm = (Get-Command npm.cmd -ErrorAction Stop).Source
$pwshCommand = Get-Command pwsh -ErrorAction SilentlyContinue
$pwsh = $null

if ($pwshCommand) {
  $pwsh = $pwshCommand.Source
}

if (-not $pwsh) {
  $pwsh = (Get-Command powershell.exe -ErrorAction Stop).Source
}

if (-not (Test-Path (Join-Path $bridgeRoot "package.json"))) {
  throw "Bridge dashboard folder not found: $bridgeRoot"
}

function Test-PortListening {
  param(
    [int]$Port
  )

  $connection = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue | Select-Object -First 1
  return $null -ne $connection
}

function ConvertTo-PowerShellSingleQuotedString {
  param(
    [string]$Value
  )

  return "'$($Value -replace "'", "''")'"
}

function Test-BridgeHealthy {
  param(
    [int]$Port
  )

  try {
    $status = Invoke-RestMethod -Uri "http://localhost:$Port/api/status" -TimeoutSec 3
    return $null -ne $status.robotConfig -and $null -ne $status.legs
  } catch {
    return $false
  }
}

function Test-ViteHealthy {
  param(
    [int]$Port
  )

  try {
    $response = Invoke-WebRequest -Uri "http://localhost:$Port/@vite/client" -UseBasicParsing -TimeoutSec 3
    return $response.StatusCode -eq 200 -and $response.Content -match "vite"
  } catch {
    return $false
  }
}

function Wait-Until {
  param(
    [scriptblock]$Condition,
    [int]$TimeoutSeconds = 20
  )

  $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
  while ((Get-Date) -lt $deadline) {
    if (& $Condition) {
      return $true
    }
    Start-Sleep -Milliseconds 500
  }

  return $false
}

Set-Location $poseDashboardRoot

if (-not (Test-Path (Join-Path $poseDashboardRoot "node_modules"))) {
  & $npm install
  if ($LASTEXITCODE -ne 0) {
    throw "npm install failed for the full-body pose dashboard."
  }
}

if (-not (Test-Path (Join-Path $bridgeRoot "node_modules"))) {
  Push-Location $bridgeRoot
  try {
    & $npm install
    if ($LASTEXITCODE -ne 0) {
      throw "npm install failed for the serial bridge dashboard."
    }
  } finally {
    Pop-Location
  }
}

if (Test-BridgeHealthy -Port $BridgePort) {
  Write-Host "Serial bridge is running on http://localhost:$BridgePort."
} elseif (Test-PortListening -Port $BridgePort) {
  throw "Port $BridgePort is already in use, but it is not a healthy Argos serial bridge. Stop that process or launch with -BridgePort <port>."
} else {
  $quotedBridgeRoot = ConvertTo-PowerShellSingleQuotedString -Value $bridgeRoot
  $quotedNpm = ConvertTo-PowerShellSingleQuotedString -Value $npm
  $bridgeCommand = "Set-Location -LiteralPath $quotedBridgeRoot; `$env:PORT='$BridgePort'; & $quotedNpm run bridge"
  Start-Process -FilePath $pwsh -ArgumentList "-NoExit", "-ExecutionPolicy", "Bypass", "-Command", $bridgeCommand | Out-Null
  Write-Host "Serial bridge starting in a new PowerShell window..."

  if (-not (Wait-Until -Condition { Test-BridgeHealthy -Port $BridgePort })) {
    throw "Serial bridge did not become healthy on port $BridgePort. Check the new PowerShell window for the bridge error."
  }

  Write-Host "Serial bridge is running on http://localhost:$BridgePort."
}

if (Test-ViteHealthy -Port $VitePort) {
  Write-Host "Full-body pose dashboard is running on http://localhost:$VitePort."
  Write-Host "Open http://localhost:$VitePort in your browser."
  return
} elseif (Test-PortListening -Port $VitePort) {
  throw "Port $VitePort is already in use, but it is not a healthy Vite dev server. Stop that process or launch with -VitePort <port>."
}

Write-Host "Starting full-body pose dashboard in this window..."
Write-Host "Open http://localhost:$VitePort in your browser."

$env:VITE_BACKEND_URL = "http://localhost:$BridgePort"
& $npm run dev -- --port $VitePort
