socat -d -d -u tcp:10.0.10.30:$2,nodelay $1,raw,echo=0,b115200 
