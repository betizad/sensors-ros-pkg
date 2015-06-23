<<<<<<< Updated upstream
#!/bin/bash
socat -d -d -u tcp:$2,nodelay,forever $1,raw,echo=0,b115200 
exit 0
=======
socat -d -d -u tcp:$2,nodelay $1,raw,echo=0,b115200 
>>>>>>> Stashed changes
