# Auto-passthrough USB devices to WSL distro
param(
    [string]$TargetDistro = "ros2",
    [string[]]$TargetDevices = @("1a86:7523"),
    [switch]$Debug = $false
)

function Test-IsAdmin {
    ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Get-TargetUsbDevices {
    $devices = @{}
    
    try {
        $output = & usbipd list 2>&1
        if ($LASTEXITCODE -ne 0) { return $devices }
        
        foreach ($line in $output) {
            if ($line -match '^(\d+-\d+)\s+([a-f0-9]{4}:[a-f0-9]{4})\s+(.+?)\s+(Not shared|Shared|Attached)\s*$') {
                $busId = $matches[1]
                $vidPid = $matches[2]
                $deviceName = $matches[3].Trim()
                $state = $matches[4]
                
                if ($TargetDevices -contains $vidPid) {
                    $devices[$busId] = @{
                        Name = $deviceName
                        VidPid = $vidPid
                        State = $state
                    }
                    
                    if ($Debug) {
                        Write-Host "[DEBUG] Found target device: $deviceName at $busId ($state)" -ForegroundColor Gray
                    }
                }
            }
        }
    } catch {
        if ($Debug) {
            Write-Host "[DEBUG] Error scanning devices: $($_.Exception.Message)" -ForegroundColor Red
        }
    }
    
    return $devices
}

function Invoke-DeviceAction {
    param($BusId, $DeviceName, $State)
    
    switch ($State) {
        "Not shared" {
            if (Test-IsAdmin) {
                Write-Host "Binding and attaching $DeviceName at $BusId..." -ForegroundColor Yellow
                
                & usbipd bind --busid $BusId 2>&1 | Out-Null
                if ($LASTEXITCODE -eq 0) {
                    & usbipd attach --wsl $TargetDistro --busid $BusId 2>&1 | Out-Null
                    if ($LASTEXITCODE -eq 0) {
                        Write-Host "[SUCCESS] Device attached to WSL" -ForegroundColor Green
                        return $true
                    } else {
                        Write-Host "[ERROR] Failed to attach device" -ForegroundColor Red
                    }
                } else {
                    Write-Host "[ERROR] Failed to bind device" -ForegroundColor Red
                }
            } else {
                Write-Host "[WARNING] Admin rights required to bind $DeviceName at $BusId" -ForegroundColor Yellow
            }
        }
        
        "Shared" {
            Write-Host "Attaching $DeviceName at $BusId..." -ForegroundColor Cyan
            
            & usbipd attach --wsl $TargetDistro --busid $BusId 2>&1 | Out-Null
            if ($LASTEXITCODE -eq 0) {
                Write-Host "[SUCCESS] Device attached to WSL" -ForegroundColor Green
                return $true
            } else {
                Write-Host "[ERROR] Failed to attach device" -ForegroundColor Red
            }
        }
        
        "Attached" {
            if ($Debug) {
                Write-Host "[DEBUG] Device $DeviceName at $BusId already attached" -ForegroundColor Gray
            }
            return $true
        }
    }
    
    return $false
}

function Compare-DeviceStates {
    param($Previous, $Current)
    
    $hasChanges = $false
    
    # Check for unplugged devices
    foreach ($busId in $Previous.Keys) {
        if (-not $Current.ContainsKey($busId)) {
            Write-Host "[UNPLUGGED] $($Previous[$busId].Name) from $busId" -ForegroundColor Red
            $hasChanges = $true
        }
    }
    
    # Check for new devices and state changes
    foreach ($busId in $Current.Keys) {
        $device = $Current[$busId]
        
        if (-not $Previous.ContainsKey($busId)) {
            Write-Host "[PLUGGED] $($device.Name) at $busId ($($device.State))" -ForegroundColor Green
            Invoke-DeviceAction -BusId $busId -DeviceName $device.Name -State $device.State
            $hasChanges = $true
        }
        elseif ($Previous[$busId].State -ne $device.State) {
            Write-Host "[STATE CHANGE] $($device.Name) at $busId changed from $($Previous[$busId].State) to $($device.State)" -ForegroundColor Cyan
            Invoke-DeviceAction -BusId $busId -DeviceName $device.Name -State $device.State
            $hasChanges = $true
        }
    }
    
    return $hasChanges
}

# Main script execution
Write-Host "USB Auto-Passthrough Monitor"
Write-Host "Target distro: $TargetDistro"
Write-Host "Target devices: $($TargetDevices -join ', ')"

if (-not (Test-IsAdmin)) {
    Write-Host "[WARNING] Not running as administrator - device binding may fail" -ForegroundColor Yellow
}

if ($Debug) {
    Write-Host "[DEBUG] Debug mode enabled" -ForegroundColor Magenta
}

Write-Host "`nPerforming initial scan..."

# Initial scan
$previousDevices = Get-TargetUsbDevices

if ($previousDevices.Count -gt 0) {
    Write-Host "Found $($previousDevices.Count) target device(s):"
    foreach ($busId in $previousDevices.Keys) {
        $device = $previousDevices[$busId]
        Write-Host "  $($device.Name) at $busId [$($device.State)]"
        
        # Process initial devices if they need action
        if ($device.State -in @("Not shared", "Shared")) {
            Invoke-DeviceAction -BusId $busId -DeviceName $device.Name -State $device.State
        }
    }
} else {
    Write-Host "No target devices found"
}

Write-Host "`nStarting monitoring... (Press Ctrl+C to stop)`n"

# Monitoring loop
try {
    while ($true) {
        Start-Sleep -Seconds 3
        
        if ($Debug) {
            Write-Host "[DEBUG] Scanning..." -ForegroundColor Gray
        }
        
        $currentDevices = Get-TargetUsbDevices
        $hasChanges = Compare-DeviceStates -Previous $previousDevices -Current $currentDevices
        
        if ($hasChanges) {
            Write-Host "" # Add blank line after events
            $previousDevices = $currentDevices
        }
    }
}
finally {
    Write-Host "Monitoring stopped."
}