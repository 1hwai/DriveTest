# DriveTest 작업 워크플로

## 로컬 검증 안내

- 코드 변경 후 GitHub Actions만 실행해 놓고 검증 완료로 취급하지 않는다.
- 사용자가 로컬에서 검증하겠다고 했거나 로컬 실행이 필요한 상황이면, Actions 결과를 기다리기 전에 사용자가 바로 실행할 수 있는 명령어를 먼저 제공한다.
- CMake 프로젝트의 기본 로컬 검증 명령은 저장소 루트에서 다음 순서로 안내한다.

```bash
git fetch origin
git switch <working-branch>
git pull --ff-only origin <working-branch>

cmake -S . -B build -DDRIVETEST_BUILD_APP=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

- 브랜치 이름은 실제 작업 브랜치로 바꿔 제시한다. 로컬에 이미 변경 사항이 있을 수 있으므로 `git reset --hard` 등 파괴적인 명령은 명시적인 요청 없이 안내하지 않는다.
- 사용자가 로컬 실행을 맡으면 그 결과를 기다리고, 로그를 받기 전에는 로컬 빌드나 테스트가 통과했다고 주장하지 않는다.
- CI 성공과 전체 앱 타깃의 로컬 빌드는 별개의 검증이다. 실제로 실행한 검증 범위를 정확히 보고한다.
