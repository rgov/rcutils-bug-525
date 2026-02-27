// repro.cpp — Demonstrate readdir() + std::vector errno pollution bug
//
// When std::vector grows past M_MMAP_THRESHOLD, glibc malloc tries mmap()
// first. Under tight RLIMIT_AS, mmap() fails (errno=ENOMEM) but malloc
// falls back to brk() which succeeds. errno is never cleared, so a
// post-loop "if (errno)" check falsely reports an error.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <malloc.h>
#include <string>
#include <sys/resource.h>
#include <vector>

int main(int argc, char *argv[]) {
    if (argc < 2 || argc > 3) {
        fprintf(stderr, "Usage: %s <directory> [headroom_kb]\n", argv[0]);
        return 2;
    }

    // glibc malloc has two allocation strategies: mmap() for large chunks
    // (default threshold ≥128KB) and brk() for smaller ones.  Our vector of
    // ~256 × 70-char strings only grows to ~16KB — well below 128KB — so it
    // would normally use brk() exclusively and the bug would never trigger.
    // Lowering the threshold to 8KB ensures vector reallocation takes the
    // mmap() path, which is where errno gets polluted on failure.
    mallopt(M_MMAP_THRESHOLD, 8192);

    // When brk() extends the heap, glibc normally requests 128KB of extra
    // padding to reduce future syscalls.  With M_TOP_PAD=0, brk() requests
    // only the actual allocation size.  Without this, after the mmap() fails
    // the brk() fallback would *also* fail (it would ask for 128KB+ of
    // headroom that doesn't exist), turning "BUG" into "OOM".
    mallopt(M_TOP_PAD, 0);

    DIR *d = opendir(argv[1]);
    if (!d) {
        fprintf(stderr, "opendir: %s\n", strerror(errno));
        return 2;
    }

    // RLIMIT_AS caps the process's total virtual address space.  We set it
    // to current VmSize + a small headroom (e.g. 20kB).  This creates the
    // exact conditions for the bug: mmap() fails (no room for a new VMA)
    // but brk() succeeds (extending the existing heap by a small amount
    // fits within the headroom).  malloc sets errno=ENOMEM on the failed
    // mmap(), falls back to brk() which succeeds, but never clears errno.
    if (argc == 3) {
        long pages;
        FILE *f = fopen("/proc/self/statm", "r");
        if (!f || fscanf(f, "%ld", &pages) != 1) {
            fprintf(stderr, "Cannot read /proc/self/statm\n");
            return 2;
        }
        fclose(f);
        long vm_kb = pages * 4;  // statm reports in pages (4kB each)
        rlim_t limit = (rlim_t)(vm_kb + atol(argv[2])) * 1024;
        struct rlimit rl = { limit, limit };
        setrlimit(RLIMIT_AS, &rl);
        fprintf(stderr, "RLIMIT_AS = %ld kB (VmSize=%ld + headroom=%s)\n",
                (long)(limit / 1024), vm_kb, argv[2]);
    }

    // The bug: readdir fills a vector; vector growth pollutes errno
    std::vector<std::string> entries;
    errno = 0;
    struct dirent *ent;
    while ((ent = readdir(d)) != nullptr)
        entries.push_back(ent->d_name);
    int saved = errno;
    closedir(d);

    if (saved != 0) {
        printf("BUG REPRODUCED: errno=%d (%s), read %zu entries\n",
               saved, strerror(saved), entries.size());
        return 0;
    }
    printf("NO BUG: errno=0, read %zu entries\n", entries.size());
    return 1;
}
