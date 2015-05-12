socat -d -d -u $1,raw,echo=0,b115200 tcp4-listen:$2,fork,reuseaddr,nodelay
