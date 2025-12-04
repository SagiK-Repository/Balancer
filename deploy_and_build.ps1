# 원격 서버에 프로젝트 배포 및 빌드 스크립트
# plink (PuTTY) 또는 SSH 키가 필요합니다

$hostname = "tbot3@192.168.0.28"
$password = "1234"
$projectPath = "src/integrated_balancer"

Write-Host "=== Remote Build Script ===" -ForegroundColor Green
Write-Host "Host: $hostname" -ForegroundColor Yellow
Write-Host ""

# 프로젝트 파일 목록 확인
if (-not (Test-Path $projectPath)) {
    Write-Host "Error: Project path not found: $projectPath" -ForegroundColor Red
    exit 1
}

Write-Host "Project found: $projectPath" -ForegroundColor Green

# plink가 있는지 확인
$plinkPath = Get-Command plink -ErrorAction SilentlyContinue
if ($plinkPath) {
    Write-Host "Using plink for SSH connection..." -ForegroundColor Green
    
    # 프로젝트를 tar로 압축
    Write-Host "Creating archive..." -ForegroundColor Yellow
    $archiveName = "integrated_balancer.tar.gz"
    
    # WSL이나 tar가 있다면 사용
    if (Get-Command tar -ErrorAction SilentlyContinue) {
        tar -czf $archiveName -C src integrated_balancer
    } else {
        Write-Host "tar not found. Please install 7-Zip or use WSL." -ForegroundColor Red
        Write-Host ""
        Write-Host "Manual steps:" -ForegroundColor Yellow
        Write-Host "1. Copy project manually:"
        Write-Host "   scp -r src/integrated_balancer $hostname`:~/catkin_ws/src/"
        Write-Host "2. SSH and build:"
        Write-Host "   ssh $hostname"
        Write-Host "   cd ~/catkin_ws"
        Write-Host "   source /opt/ros/noetic/setup.bash"
        Write-Host "   catkin_make"
        exit 1
    }
    
    # plink로 파일 전송 및 빌드
    Write-Host "Uploading project..." -ForegroundColor Yellow
    echo y | plink -pw $password $hostname "mkdir -p ~/catkin_ws/src"
    
    # pscp로 파일 전송
    if (Get-Command pscp -ErrorAction SilentlyContinue) {
        echo y | pscp -pw $password -r $projectPath $hostname`:~/catkin_ws/src/
    } else {
        Write-Host "pscp not found. Please install PuTTY tools." -ForegroundColor Red
        exit 1
    }
    
    Write-Host "Building on remote server..." -ForegroundColor Yellow
    $buildCmd = "cd ~/catkin_ws && source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null && catkin_make"
    echo y | plink -pw $password $hostname $buildCmd
    
} else {
    Write-Host "plink not found. Using manual instructions..." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Please run the following commands manually:" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "1. Copy project to remote server:" -ForegroundColor White
    Write-Host "   scp -r src/integrated_balancer $hostname`:~/catkin_ws/src/" -ForegroundColor Gray
    Write-Host ""
    Write-Host "2. SSH to remote server:" -ForegroundColor White
    Write-Host "   ssh $hostname" -ForegroundColor Gray
    Write-Host ""
    Write-Host "3. Build the project:" -ForegroundColor White
    Write-Host "   cd ~/catkin_ws" -ForegroundColor Gray
    Write-Host "   source /opt/ros/noetic/setup.bash" -ForegroundColor Gray
    Write-Host "   catkin_make" -ForegroundColor Gray
    Write-Host ""
}

Write-Host "Done!" -ForegroundColor Green



