# SSH 원격 프로젝트 업로드 및 빌드 가이드

이 문서는 SSH를 통해 원격 서버에 프로젝트 파일을 업로드하고 빌드하는 일반적인 방법을 설명합니다.

## 목차

1. [개요](#개요)
2. [필수 요구사항](#필수-요구사항)
3. [기본 구조](#기본-구조)
4. [핵심 함수 구현](#핵심-함수-구현)
5. [사용 예제](#사용-예제)
6. [실전 패턴](#실전-패턴)
7. [문제 해결](#문제-해결)

---

## 개요

로컬에서 개발한 코드를 원격 서버(로봇, 임베디드 시스템 등)에 업로드하고 빌드하는 작업을 자동화하는 Python 스크립트 작성 방법을 다룹니다.

### 주요 기능

- SSH 연결 및 인증
- 파일 업로드 (SFTP)
- 원격 명령 실행
- 프로세스 관리 (종료/시작)
- 빌드 및 검증

---

## 필수 요구사항

### Python 패키지

```bash
pip install paramiko
```

### 원격 서버 정보

스크립트 상단에 다음 정보를 설정합니다:

```python
HOSTNAME = "192.168.0.28"  # 원격 서버 IP 주소
USERNAME = "tbot3"          # SSH 사용자명
PASSWORD = "1234"           # SSH 비밀번호 (또는 SSH 키 사용)
```

> **보안 참고**: 프로덕션 환경에서는 비밀번호 대신 SSH 키를 사용하거나, 환경 변수나 설정 파일에서 읽어오는 것을 권장합니다.

---

## 기본 구조

### 스크립트 템플릿

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 서버 프로젝트 업로드 및 빌드 스크립트
"""

import paramiko
import sys
import time

# 원격 서버 정보
HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

def create_ssh_client():
    """SSH 클라이언트 생성"""
    pass

def upload_file(sftp, local_path, remote_path):
    """파일 업로드"""
    pass

def run_command(client, command, timeout=60, show_output=True):
    """원격 명령 실행"""
    pass

def main():
    """메인 함수"""
    pass

if __name__ == "__main__":
    # Windows 인코딩 처리
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())
```

---

## 핵심 함수 구현

### 1. SSH 클라이언트 생성

```python
def create_ssh_client():
    """SSH 클라이언트 생성 및 연결"""
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"Connecting to {USERNAME}@{HOSTNAME}...")
        client.connect(HOSTNAME, username=USERNAME, password=PASSWORD, timeout=10)
        print("✅ Connected successfully!\n")
        return client
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None
```

**설명:**
- `AutoAddPolicy()`: 호스트 키 자동 추가 (개발 환경용)
- `timeout=10`: 연결 타임아웃 10초
- 프로덕션에서는 호스트 키 검증을 엄격하게 설정

### 2. 파일 업로드 (방법 1: SFTP put)

```python
def upload_file(sftp, local_path, remote_path):
    """파일 업로드 (SFTP put 사용)"""
    try:
        sftp.put(local_path, remote_path)
        print(f"   ✅ 업로드 완료: {remote_path}")
        return True
    except Exception as e:
        print(f"   ❌ 업로드 실패: {e}")
        return False
```

### 3. 파일 업로드 (방법 2: 직접 쓰기)

```python
def upload_file_content(sftp, content, remote_path):
    """파일 내용 직접 업로드 (문자열 내용)"""
    try:
        with sftp.file(remote_path, 'w') as remote_file:
            remote_file.write(content)
        print(f"   ✅ 업로드 완료: {remote_path}")
        return True
    except Exception as e:
        print(f"   ❌ 업로드 실패: {e}")
        return False
```

**사용 예:**
```python
# 로컬 파일 읽기
with open('local_file.cpp', 'r', encoding='utf-8') as f:
    content = f.read()

# 원격에 쓰기
sftp = client.open_sftp()
upload_file_content(sftp, content, '/remote/path/file.cpp')
sftp.close()
```

### 4. 원격 명령 실행

```python
def run_command(client, command, timeout=60, show_output=True):
    """원격 명령 실행 및 출력 처리"""
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        start_time = time.time()
        
        output = ""
        error = ""
        
        while True:
            # 타임아웃 체크
            if time.time() - start_time > timeout:
                stdout.channel.close()
                stderr.channel.close()
                return False, output, f"Timeout after {timeout}s"
            
            # stdout 읽기
            if stdout.channel.recv_ready():
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                output += chunk
                if show_output:
                    print(chunk, end='', flush=True)
            
            # stderr 읽기
            if stderr.channel.recv_stderr_ready():
                chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                error += chunk
                if show_output:
                    print(chunk, end='', flush=True, file=sys.stderr)
            
            # 명령 완료 확인
            if stdout.channel.exit_status_ready():
                exit_status = stdout.channel.recv_exit_status()
                
                # 남은 출력 읽기
                while stdout.channel.recv_ready():
                    chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                    output += chunk
                    if show_output:
                        print(chunk, end='', flush=True)
                
                while stderr.channel.recv_stderr_ready():
                    chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                    error += chunk
                    if show_output:
                        print(chunk, end='', flush=True, file=sys.stderr)
                break
            
            time.sleep(0.05)  # CPU 부하 감소
        
        return exit_status == 0, output, error
    except Exception as e:
        return False, "", str(e)
```

**설명:**
- 실시간 출력 스트리밍
- 타임아웃 처리
- stdout/stderr 분리 처리
- UTF-8 인코딩 처리

---

## 사용 예제

### 예제 1: 단일 파일 업로드 및 빌드

```python
def main():
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 1. 파일 업로드
        sftp = client.open_sftp()
        upload_file(sftp, 
                   'src/my_project/main.cpp', 
                   '/home/user/project/src/main.cpp')
        sftp.close()
        
        # 2. 빌드
        build_cmd = "cd /home/user/project && make"
        success, output, error = run_command(client, build_cmd, timeout=120)
        
        if success:
            print("✅ 빌드 완료!")
        else:
            print(f"❌ 빌드 실패: {error}")
            return 1
        
        return 0
    finally:
        client.close()
```

### 예제 2: 여러 파일 업로드

```python
def main():
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        sftp = client.open_sftp()
        
        # 업로드할 파일 목록
        files_to_upload = [
            ('src/file1.cpp', '/remote/path/file1.cpp'),
            ('src/file2.h', '/remote/path/file2.h'),
            ('config/settings.json', '/remote/path/settings.json'),
        ]
        
        print("파일 업로드 중...")
        for local_path, remote_path in files_to_upload:
            if not upload_file(sftp, local_path, remote_path):
                print(f"❌ 업로드 실패: {local_path}")
                return 1
        
        sftp.close()
        print("✅ 모든 파일 업로드 완료!")
        
        return 0
    finally:
        client.close()
```

### 예제 3: 프로세스 관리 (종료 후 시작)

```python
def main():
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 1. 기존 프로세스 종료
        print("기존 프로세스 종료 중...")
        run_command(client, "pkill -f my_service", timeout=3, show_output=False)
        time.sleep(2)  # 프로세스 종료 대기
        
        # 2. 파일 업로드 및 빌드
        # ... (업로드 및 빌드 코드)
        
        # 3. 새 프로세스 시작
        print("새 프로세스 시작 중...")
        start_cmd = "cd /home/user/project && nohup ./my_service > /tmp/service.log 2>&1 &"
        client.exec_command(start_cmd, timeout=2)
        time.sleep(3)  # 프로세스 시작 대기
        
        # 4. 실행 확인
        success, output, error = run_command(
            client, 
            "ps aux | grep my_service | grep -v grep", 
            timeout=3, 
            show_output=False
        )
        
        if success and output.strip():
            print("✅ 프로세스 실행 중")
        else:
            print("❌ 프로세스 실행 실패")
            return 1
        
        return 0
    finally:
        client.close()
```

---

## 실전 패턴

### 패턴 1: 환경 변수 설정 및 빌드 (ROS 예제)

```python
def main():
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 현재 IP 확인
        success, output, error = run_command(
            client, 
            "hostname -I | awk '{print $1}'", 
            timeout=3, 
            show_output=False
        )
        current_ip = output.strip() if success else "192.168.0.28"
        
        # 환경 변수 설정
        setup_cmd = "cd ~/catkin_ws && source devel/setup.bash"
        env_cmd = f"export ROS_MASTER_URI='http://{current_ip}:11311' && " \
                  f"export ROS_IP='{current_ip}' && {setup_cmd}"
        
        # 빌드
        build_cmd = f"{env_cmd} && cd ~/catkin_ws && catkin_make"
        success, output, error = run_command(client, build_cmd, timeout=120)
        
        return 0 if success else 1
    finally:
        client.close()
```

### 패턴 2: 파일 내용 직접 쓰기 (메시지 파일 등)

```python
def main():
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 로컬 파일 읽기
        with open('config/message.msg', 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 원격에 쓰기
        sftp = client.open_sftp()
        remote_path = '/home/user/project/msg/message.msg'
        with sftp.file(remote_path, 'w') as remote_file:
            remote_file.write(content)
        sftp.close()
        
        print("✅ 파일 업로드 완료")
        return 0
    finally:
        client.close()
```

### 패턴 3: 빌드 결과 검증

```python
def main():
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 빌드 실행
        build_cmd = "cd /home/user/project && make"
        success, output, error = run_command(client, build_cmd, timeout=120)
        
        if not success:
            print(f"❌ 빌드 실패: {error}")
            return 1
        
        # 빌드 결과 확인
        check_cmd = "ls -lh /home/user/project/bin/my_program 2>/dev/null || echo '파일 없음'"
        success, output, error = run_command(client, check_cmd, timeout=5)
        
        if '파일 없음' in output:
            print("❌ 빌드 결과 파일이 없습니다")
            return 1
        
        print("✅ 빌드 완료 및 검증 성공")
        print(output)
        return 0
    finally:
        client.close()
```

### 패턴 4: 백그라운드 프로세스 시작 및 로그 확인

```python
def main():
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 백그라운드로 프로세스 시작
        start_cmd = "cd /home/user/project && nohup ./my_service > /tmp/service.log 2>&1 &"
        client.exec_command(start_cmd, timeout=2)
        time.sleep(3)
        
        # 프로세스 실행 확인
        for i in range(5):
            success, output, error = run_command(
                client, 
                "ps aux | grep my_service | grep -v grep", 
                timeout=3, 
                show_output=False
            )
            if success and output.strip():
                print("✅ 프로세스 실행 중")
                break
            time.sleep(1)
        else:
            print("❌ 프로세스 실행 확인 실패")
            return 1
        
        # 로그 확인 (선택적)
        success, output, error = run_command(
            client, 
            "tail -n 20 /tmp/service.log", 
            timeout=5
        )
        
        return 0
    finally:
        client.close()
```

---

## 문제 해결

### 1. 연결 타임아웃

**증상:** `Connection failed: timeout`

**해결:**
- 네트워크 연결 확인
- 방화벽 설정 확인
- `timeout` 값 증가

```python
client.connect(HOSTNAME, username=USERNAME, password=PASSWORD, timeout=30)
```

### 2. 파일 업로드 실패

**증상:** `업로드 실패: Permission denied`

**해결:**
- 원격 경로 권한 확인
- 디렉토리 존재 여부 확인

```python
# 디렉토리 생성
run_command(client, f"mkdir -p {os.path.dirname(remote_path)}", timeout=5)
```

### 3. 명령 실행 타임아웃

**증상:** 빌드 중 타임아웃 발생

**해결:**
- 타임아웃 값 증가
- 백그라운드 실행 고려

```python
# 타임아웃 증가
success, output, error = run_command(client, build_cmd, timeout=300)

# 또는 백그라운드 실행
client.exec_command(f"nohup {build_cmd} > /tmp/build.log 2>&1 &", timeout=2)
```

### 4. 인코딩 문제 (Windows)

**증상:** 한글 출력 깨짐

**해결:**
- 스크립트 시작 부분에 인코딩 설정 추가

```python
if sys.platform == 'win32':
    import io
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
    except:
        pass
```

### 5. SSH 키 인증 사용

비밀번호 대신 SSH 키 사용:

```python
def create_ssh_client():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    # SSH 키 사용
    private_key = paramiko.RSAKey.from_private_key_file('/path/to/private_key')
    
    try:
        client.connect(
            HOSTNAME, 
            username=USERNAME, 
            pkey=private_key,  # 비밀번호 대신 키 사용
            timeout=10
        )
        return client
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None
```

---

## 완전한 예제 스크립트

다음은 실제 사용 가능한 완전한 예제입니다:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
원격 서버 프로젝트 업로드 및 빌드 스크립트
"""

import paramiko
import sys
import time
import os

# 원격 서버 정보
HOSTNAME = "192.168.0.28"
USERNAME = "tbot3"
PASSWORD = "1234"

# 프로젝트 경로
REMOTE_PROJECT_PATH = "/home/user/my_project"
LOCAL_PROJECT_PATH = "./src"

def create_ssh_client():
    """SSH 클라이언트 생성"""
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"Connecting to {USERNAME}@{HOSTNAME}...")
        client.connect(HOSTNAME, username=USERNAME, password=PASSWORD, timeout=10)
        print("✅ Connected successfully!\n")
        return client
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None

def upload_file(sftp, local_path, remote_path):
    """파일 업로드"""
    try:
        # 디렉토리 생성
        remote_dir = os.path.dirname(remote_path)
        sftp.makedirs(remote_dir, exist_ok=True)
        
        sftp.put(local_path, remote_path)
        print(f"   ✅ 업로드 완료: {remote_path}")
        return True
    except Exception as e:
        print(f"   ❌ 업로드 실패: {e}")
        return False

def run_command(client, command, timeout=60, show_output=True):
    """원격 명령 실행"""
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        start_time = time.time()
        
        output = ""
        error = ""
        
        while True:
            if time.time() - start_time > timeout:
                stdout.channel.close()
                stderr.channel.close()
                return False, output, f"Timeout after {timeout}s"
            
            if stdout.channel.recv_ready():
                chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                output += chunk
                if show_output:
                    print(chunk, end='', flush=True)
            
            if stderr.channel.recv_stderr_ready():
                chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                error += chunk
                if show_output:
                    print(chunk, end='', flush=True, file=sys.stderr)
            
            if stdout.channel.exit_status_ready():
                exit_status = stdout.channel.recv_exit_status()
                while stdout.channel.recv_ready():
                    chunk = stdout.channel.recv(4096).decode('utf-8', errors='ignore')
                    output += chunk
                    if show_output:
                        print(chunk, end='', flush=True)
                while stderr.channel.recv_stderr_ready():
                    chunk = stderr.channel.recv_stderr(4096).decode('utf-8', errors='ignore')
                    error += chunk
                    if show_output:
                        print(chunk, end='', flush=True, file=sys.stderr)
                break
            
            time.sleep(0.05)
        
        return exit_status == 0, output, error
    except Exception as e:
        return False, "", str(e)

def main():
    """메인 함수"""
    print("="*80)
    print("원격 서버 프로젝트 업로드 및 빌드")
    print("="*80)
    print()
    
    client = create_ssh_client()
    if not client:
        return 1
    
    try:
        # 1. 기존 프로세스 종료
        print("1. 기존 프로세스 종료 중...")
        run_command(client, "pkill -f my_service", timeout=3, show_output=False)
        time.sleep(2)
        print("   ✅ 프로세스 종료 완료\n")
        
        # 2. 파일 업로드
        print("2. 파일 업로드 중...")
        sftp = client.open_sftp()
        
        files_to_upload = [
            ('main.cpp', 'src/main.cpp'),
            ('config.h', 'include/config.h'),
        ]
        
        for local_file, remote_file in files_to_upload:
            local_path = os.path.join(LOCAL_PROJECT_PATH, local_file)
            remote_path = os.path.join(REMOTE_PROJECT_PATH, remote_file)
            if not upload_file(sftp, local_path, remote_path):
                print(f"❌ 업로드 실패: {local_file}")
                return 1
        
        sftp.close()
        print()
        
        # 3. 빌드
        print("3. 빌드 중...")
        build_cmd = f"cd {REMOTE_PROJECT_PATH} && make"
        success, output, error = run_command(client, build_cmd, timeout=120)
        
        if not success:
            print(f"\n❌ 빌드 실패!")
            print(f"Error: {error}")
            return 1
        
        print("\n   ✅ 빌드 완료!\n")
        
        # 4. 빌드 결과 확인
        print("4. 빌드 결과 확인 중...")
        check_cmd = f"ls -lh {REMOTE_PROJECT_PATH}/bin/my_program 2>/dev/null || echo '파일 없음'"
        success, output, error = run_command(client, check_cmd, timeout=5)
        print()
        
        # 5. 프로세스 시작
        print("5. 프로세스 시작 중...")
        start_cmd = f"cd {REMOTE_PROJECT_PATH} && nohup ./bin/my_program > /tmp/service.log 2>&1 &"
        client.exec_command(start_cmd, timeout=2)
        time.sleep(3)
        print("   ✅ 프로세스 시작 완료\n")
        
        print("="*80)
        print("✅ 모든 작업 완료!")
        print("="*80)
        
        return 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        try:
            if client:
                client.close()
        except:
            pass

if __name__ == "__main__":
    if sys.platform == 'win32':
        import io
        try:
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')
        except:
            pass
    
    sys.exit(main())
```

---

## 요약

1. **SSH 연결**: `paramiko.SSHClient()` 사용
2. **파일 업로드**: SFTP를 통해 `put()` 또는 직접 쓰기
3. **명령 실행**: `exec_command()`로 실행하고 실시간 출력 처리
4. **프로세스 관리**: `pkill`로 종료, `nohup`으로 시작
5. **에러 처리**: try-except와 타임아웃 처리 필수

이 패턴을 사용하면 다양한 원격 배포 시나리오에 적용할 수 있습니다.

