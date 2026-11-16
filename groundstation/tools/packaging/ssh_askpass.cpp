// Windows OpenSSH askpass：密码只读取当前进程环境，不写磁盘或命令行。
#include <cstdio>
#include <cstdlib>
#include <cstring>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif
int main(int argc,char **argv) {
    if(argc < 2 || !std::strstr(argv[1],"assword")) return 1;
    const char *password=std::getenv("GCS_SSH_PASSWORD");
    if(!password) return 1;
#ifdef _WIN32
    _setmode(_fileno(stdout),_O_BINARY);
#endif
    std::fputs(password,stdout);std::fputc('\n',stdout);
    return std::fflush(stdout)==0?0:1;
}
