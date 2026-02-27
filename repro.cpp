// repro.cpp — Demonstrate readdir() + std::vector errno pollution bug
//
// When readdir() fills a std::vector<std::string> and the vector's backing
// store grows past M_MMAP_THRESHOLD, glibc's malloc first tries mmap().
// Under tight RLIMIT_AS, mmap() fails (errno = ENOMEM) but malloc falls
// back to sbrk/brk which succeeds. errno is never cleared, so a post-loop
// "if (errno != 0)" check falsely reports an error.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <malloc.h>
#include <string>
#include <sys/resource.h>
#include <vector>

static long read_proc_value(const char *key) {
    FILE *f = fopen("/proc/self/status", "r");
    if (!f) return -1;
    char line[256];
    long val = -1;
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, key, strlen(key)) == 0) {
            sscanf(line + strlen(key), " %ld", &val);
            break;
        }
    }
    fclose(f);
    return val;
}

int main(int argc, char *argv[]) {
    if (argc < 2 || argc > 3) {
        fprintf(stderr, "Usage: %s <directory> [rlimit_headroom_kb]\n", argv[0]);
        return 2;
    }

    // Lower mmap threshold so that a 16KB vector backing store (512 × 32)
    // triggers mmap inside glibc's sysmalloc, rather than needing thousands
    // of files to reach the default 128KB threshold.
    mallopt(M_MMAP_THRESHOLD, 8192);

    // Zero the top pad so sbrk only requests (nb - top_chunk_free), not
    // the default 128KB extra. This widens the window where mmap fails
    // (needs full allocation as new VM) but sbrk succeeds (needs less).
    mallopt(M_TOP_PAD, 0);

    DIR *d = opendir(argv[1]);
    if (!d) {
        fprintf(stderr, "opendir(%s): %s\n", argv[1], strerror(errno));
        return 2;
    }

    // If headroom specified, set RLIMIT_AS after all libraries are loaded
    // and opendir is done, but before the readdir loop.
    if (argc == 3) {
        long headroom_kb = atol(argv[2]);
        long vm_now = read_proc_value("VmSize:");
        if (vm_now < 0) {
            fprintf(stderr, "Cannot read VmSize\n");
            closedir(d);
            return 2;
        }
        rlim_t limit = (rlim_t)(vm_now + headroom_kb) * 1024;
        struct rlimit rl = { limit, limit };
        if (setrlimit(RLIMIT_AS, &rl) != 0) {
            fprintf(stderr, "setrlimit failed: %s\n", strerror(errno));
            closedir(d);
            return 2;
        }
        fprintf(stderr, "RLIMIT_AS = %ld kB (VmSize=%ld + headroom=%ld)\n",
                (long)(limit / 1024), vm_now, headroom_kb);
    }

    std::vector<std::string> entries;
    errno = 0;

    try {
        struct dirent *ent;
        while ((ent = readdir(d)) != nullptr) {
            entries.push_back(ent->d_name);
        }
    } catch (const std::bad_alloc &) {
        fprintf(stderr, "ALLOC FAILURE: caught std::bad_alloc after %zu entries\n",
                entries.size());
        closedir(d);
        return 2;
    }

    int saved_errno = errno;
    closedir(d);

    long vmpeak = read_proc_value("VmPeak:");
    if (vmpeak > 0)
        printf("VmPeak: %ld kB\n", vmpeak);

    if (saved_errno != 0) {
        printf("BUG REPRODUCED: errno=%d (%s), read %zu entries\n",
               saved_errno, strerror(saved_errno), entries.size());
        return 0;  // 0 = bug reproduced (success for our purposes)
    }

    printf("NO BUG: errno=0, read %zu entries\n", entries.size());
    return 1;  // 1 = no bug seen
}
