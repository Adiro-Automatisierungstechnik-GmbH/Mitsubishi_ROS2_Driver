# Get the directory path where the script is located
$scriptDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path

# Define the relative path to your docker-compose.yml file
$dockerComposeRelativePath = "docker-compose.yml"
$dockerComposeFilePath = Join-Path -Path $scriptDirectory -ChildPath $dockerComposeRelativePath

# Check if docker daemon is running
# Execute a Docker command to check if Docker daemon is running
$dockerVersionOutput = docker version 2>&1

# Check if there's an error in the output
if ($LASTEXITCODE -eq 0) 
{
    Write-Host "Docker daemon is running."
    Write-Host "Starting the docker containers."
} 
else {
    Write-Host "Docker daemon is not running or encountered an error:"
    Write-Host $dockerVersionOutput
    exit 1
}

# Execute docker-compose up command
try {
    docker-compose -f $dockerComposeFilePath up -d --build
    Write-Host "Successfully started the containers." -ForegroundColor Green
} catch {
    Write-Host "Error occurred while executing docker-compose up command:`n$_" -ForegroundColor Red
    exit 1
}
