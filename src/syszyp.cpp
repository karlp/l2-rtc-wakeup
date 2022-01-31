// zyps' syscalls, from https://paste.jvnv.net/view/KCh40
// modified for ITM by Karl Palsson
// Bare minimum required to get some ITM printf action.
// Some extra commented out functions from original source left in case
// something starts needing them.
// In other C-land projects, -lnosys handles this, but that didn't work for
// this little c++ project...

#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <cortex_m/debug.h>

extern char _bss_end;
char* heap = &_bss_end;
extern "C" char* _sbrk(int increment) {
   char* r = heap;
   heap += increment;
   return r; 
}


extern "C" __attribute__((weak)) int _write(int file, char* ptr, int len) {
	const auto STIMULUS_STDIO = 0;
        int i;

        if (file == STDOUT_FILENO || file == STDERR_FILENO) {
                for (i = 0; i < len; i++) {
                        if (ptr[i] == '\n') {
                                ITM->stim_blocking(STIMULUS_STDIO, '\r');
                        }
                        ITM->stim_blocking(STIMULUS_STDIO, ptr[i]);
                }
                return i;
        }
        errno = EIO;
        return -1;
}

extern "C" int _read(int file, char* buf, int len) {
    return 0;
}
extern "C" int _close(int file) {
    return -1;
}
extern "C" int _fstat(int file, struct stat* st) {
    return 0;
}
extern "C" int _isatty(int file) {
    return 1;
}
extern "C" int _lseek(int file, int offset, int whence) {
    return 0;
}
//extern "C" int _getpid() {
//    return 1;
//}
//extern "C" int _kill(int pid, int sig) {
//    return -1;
//}
//extern "C" int _exit() {
//    while(1) {}
//}
//extern "C" int _fini() {
//    return 0;
//}
//
//namespace std {
//    void __throw_length_error(char const*) { while(1) {} }
//    void __throw_bad_alloc() { while(1) {} }
//}
//
//void* operator new(std::size_t sz) {
//    //printf("malloc\n");
//    //return (void*)0x20002000;
//    return malloc(sz);
//}
//
//void operator delete(void* p) {
//    //printf("free\n");
//    free(p);
//}
//
//void operator delete(void* p, std::size_t n) {
//    //printf("free\n");
//    free(p);
//}
