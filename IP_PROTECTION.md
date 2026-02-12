# IP Protection Notice

This repository has been sanitized to protect intellectual property and personal information.

## Sanitization Actions

1. **Personal Information Removed**:
   - All personal email addresses replaced with `maintainer@ugv-autonomy.local`
   - Author names replaced with "UGV Development Team"
   - Git commit history sanitized via `.mailmap`

2. **Generic Identifiers Used**:
   - Maintainer: UGV Development Team
   - Email: maintainer@ugv-autonomy.local

## Viewing Sanitized Git History

The `.mailmap` file ensures that `git log` and `git shortlog` display generic team identities instead of personal information:

```bash
# View sanitized commit history
git log --use-mailmap

# View sanitized contributor list
git shortlog --summary --numbered --use-mailmap
```

## Before Sharing

Before sharing this repository publicly or with third parties:

1. ✅ Verify no personal information in code comments
2. ✅ Check for hardcoded credentials or API keys
3. ✅ Review commit messages for sensitive information
4. ✅ Ensure `.mailmap` is committed
5. ✅ Consider using `git filter-repo` for complete history rewrite if needed

## Note

The `.mailmap` file provides display-level sanitization. For complete history rewriting (removing personal information from git objects), consider using `git filter-repo` or `BFG Repo-Cleaner`.
