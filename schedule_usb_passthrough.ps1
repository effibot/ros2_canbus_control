# Create scheduled task for USB auto-passthrough
$TaskName = "USB Auto-Passthrough"
$ScriptPath = "$env:USERPROFILE\Documents\auto_passthrough.ps1"
$User = $env:USERNAME

# Remove existing task if it exists
Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false -ErrorAction SilentlyContinue

# Create new task
$Action = New-ScheduledTaskAction -Execute "powershell.exe" -Argument "-ExecutionPolicy Bypass -WindowStyle Hidden -File `"$ScriptPath`""
$Trigger = New-ScheduledTaskTrigger -AtStartup
$Settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable
$Principal = New-ScheduledTaskPrincipal -UserId $User -LogonType Interactive -RunLevel Highest

Register-ScheduledTask -TaskName $TaskName -Action $Action -Trigger $Trigger -Settings $Settings -Principal $Principal -Description "Auto-attach USB devices to WSL on startup"

Write-Host "Task '$TaskName' created successfully!" -ForegroundColor Green
Write-Host "The script will run automatically at Windows startup with admin privileges." -ForegroundColor Green