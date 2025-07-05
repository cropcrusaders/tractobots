<#
Launch Tractobots Instant Dashboard and Gazebo Simulation in ROS2 (PowerShell)
Usage: .\launch_dashboard.ps1 [-ros2SetupScript <path>]
#>
param(
    [string]$ros2SetupScript = "$env:USERPROFILE\ros2_humble\local_setup.ps1"
)

Write-Host "🔧 Sourcing ROS2 setup script: $ros2SetupScript"
. $ros2SetupScript

Write-Host "🚀 Launching Gazebo simulation..."
Start-Process -NoNewWindow -FilePath "ros2" -ArgumentList "launch tractobots_gazebo tractobots_gazebo.launch.py"

Write-Host "🌐 Launching Instant Dashboard..."
Start-Process -NoNewWindow -FilePath "python" -ArgumentList ".\instant_dashboard.py"
