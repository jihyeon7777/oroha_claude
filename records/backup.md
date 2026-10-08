# 백업 기록

| 날짜 | 무엇 | 어디 | 확인 |
|---|---|---|---|
| 2026-10-08 | git 저장소 전체(main `9a66701`까지) | GitHub `jihyeon7777/oroha_claude` (public), `origin` | `git ls-remote origin` = 로컬 main |
| 2026-10-08 | 논문 묶음 `paper_export/20261008_162817` → `oroha_paper_export_20261008_162817.tar.gz` (16.5 MB) | GitHub 릴리스 [`data-20261008`](https://github.com/jihyeon7777/oroha_claude/releases/tag/data-20261008) (사용자 업로드, prerelease) · Pi `data/backup/20261008/` | sha256 `1673b9a6077aa0038955c85d7a302dd2963c80ba802319bed5d708f84d35ac5d` |
| 2026-10-08 | 모든 `data/runs/*/bag` + `data/tests/*/bag*` → `oroha_raw_bags_20261008.tar.gz` (37.2 MB) | 같음 | sha256 `6078cec2ab3ee5238c8d32428da8416b26e3edb5d95e7b53492ae7f1a0504803` |

- 릴리스 올리기: 사용자 터미널에서 `python3 setup/release_upload.py data/backup/20261008/notes.md data/backup/20261008/*.tar.gz data/backup/20261008/SHA256SUMS` 또는 GitHub 웹 Releases → Draft a new release(tag `data-20261008`) → 파일 첨부.
- 복원: 저장소 clone → 워크스페이스 루트에서 `tar xzf oroha_raw_bags_20261008.tar.gz`, `paper_export/`에서 `tar xzf oroha_paper_export_….tar.gz` → `sha256sum -c SHA256SUMS`.

- **확인 2026-10-08**: 릴리스 자산 3개(16539972·37229304·205 B)를 공개 주소로 다시 내려받아 `sha256sum -c SHA256SUMS` 두 파일 OK, 논문 묶음 560 항목, 원본 녹화 bag 69개.
