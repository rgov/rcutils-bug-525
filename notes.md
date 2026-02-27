# Notes

## POSIX errno semantics

Two key rules from the [POSIX `errno` specification](https://pubs.opengroup.org/onlinepubs/9699919799/functions/errno.html):

> The setting of errno after a **successful** call to a function is
> **unspecified** unless the description of that function specifies that
> errno shall not be modified.

> No function in this volume of POSIX.1-2017 shall set errno to **0**.

This means `malloc` (or any successful call) is allowed to leave `errno`
dirty, and glibc is prohibited from clearing it to 0. The glibc maintainers
consider the mmap-fail-then-brk-succeed path "working as designed" because
callers should only inspect `errno` after a call that *returned an error
indication* (e.g., `readdir()` returning `NULL`).

## Where it happens in glibc (2.41, `malloc/malloc.c`)

| Step | Lines | What happens |
|------|-------|--------------|
| mmap attempt | 2565-2582 | `nb >= mp_.mmap_threshold` -> `sysmalloc_mmap()` -> kernel sets `errno=ENOMEM` |
| brk fallback | 2709-2715 | `MORECORE(size)` (sbrk) succeeds -- errno not cleared |
| success return | 2940 | returns valid pointer with `errno` still `ENOMEM` |

Line 2944 has `__set_errno(ENOMEM)` but that's the *failure* path only.

## The footgun pattern

```c
errno = 0;
while ((ent = readdir(d)) != NULL)
    vec.push_back(ent->d_name);  // vector growth -> malloc -> mmap fail -> errno=ENOMEM
if (errno)  // WRONG: errno is stale from malloc, not from readdir
```

The correct pattern is to reset `errno = 0` immediately before the final
check (or before each `readdir()` call).
