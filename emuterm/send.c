/* -*- C -*- */

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#define WPIPE "\\\\.\\pipe\\oskTERM"

int main(int argc, char *argv[]) {
    HANDLE hPipe = INVALID_HANDLE_VALUE;
    char input[80];
    
    printf("Running as %s\n", argv[0]);
    
    if( !strcmp(argv[0],"send") ) {
        
        hPipe = CreateNamedPipe(WPIPE,
                                PIPE_ACCESS_DUPLEX,
                                PIPE_TYPE_BYTE | PIPE_WAIT,
                                PIPE_UNLIMITED_INSTANCES,
                                512, 512, 0, NULL);
                              
        if (hPipe == INVALID_HANDLE_VALUE) 
        {
            printf(TEXT("CreateNamedPipe failed, GLE=%d.\n"), GetLastError()); 
            return -1;
        }
    
        gets(input);
        write(hPipe, input, strlen(input));
    
        CloseHandle(hPipe);
    }
    else {
        hPipe = open(WPIPE, 0666);
        if( hPipe == INVALID_HANDLE_VALUE ) {
            perror("open");
            return -1;
        }
        read(hPipe, input, 80);
        printf("Read: '%s'\n", input);
        
        CloseHandle(hPipe);
    }
    
    return 0;
}

