# Define the name of the container to attach to
$containerName = "ros2-mitsubishi-moveit"

# Check if the container is running
$containerState = (docker inspect -f "{{.State.Running}}" $containerName) 2>&1

if ($LASTEXITCODE -eq 0 -and $containerState -eq "true") {
    # Attach to the container
    docker exec -it $containerName bash
} elseif ($LASTEXITCODE -eq 0 -and $containerState -eq "false") {
    Write-Host "Error: Container '$containerName' is not running." -ForegroundColor Yellow
} else {
    Write-Host "Error: Failed to inspect container '$containerName'."
    Write-Host $containerState -ForegroundColor Red
}