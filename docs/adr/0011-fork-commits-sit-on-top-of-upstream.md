# 0011. Fork commits sit on top of upstream

## Status

Accepted.

## Context

ADR 0002 keeps the fork's changes small and contained so that upstream's work can keep coming
in. It assumed that would happen through `git merge upstream/master`. Merging works, but every
sync then leaves a merge commit in master, and over time the history becomes upstream and fork
interleaved. Reading it no longer answers the question that matters most for a fork: what does
this fork change, on top of which upstream version?

## Decision

Master is upstream's master with the fork's commits replayed on top of it, and the fork's
commits are linear. Syncing with upstream means rebasing the fork's commits onto the new
`upstream/master` and force-pushing master with `--force-with-lease`. Upstream's own history,
merge commits included, stays exactly as upstream has it underneath.

Fork changes still land through pull requests, rebase-merged, so they join the top of the
stack as they were reviewed.

ADR 0002 stands, with this replacing its merge step. Its containment rules matter more under a
rebase, not less: every fork commit that touches an upstream file is replayed against each new
upstream version, so a conflict in a widely spread change would be paid once per commit rather
than once per sync.

## Consequences

- `git log upstream/master..master` is the complete list of what the fork changes, in order,
  at any time.
- The fork's commit hashes change on every sync. Links to a fork commit by hash go stale; links
  to pull requests do not. Local branches and worktrees based on an older master need
  `git rebase --onto origin/master <old-master>` after a sync.
- A force push to master is part of the routine. `--force-with-lease` keeps it from discarding
  anything pushed in the meantime.
- Conflicts surface commit by commit during the rebase, against the commit that introduced the
  hunk. The checklist ADR 0002 keeps is still the list of places to look.
