#!/bin/bash

# add to repo

git add . 

# git commit repository

git commit -m "{$1}"

# pushing to remote repo

git push
# git pull repository
git pull

# Store the password in a variable

# copy file to server using sshpass
# chnage the source path /mnt/c/Users/kahaa/Documents/Github/RDNT/website/* according to your soruce file path
sshpass -f ../../pass.txt scp -r /mnt/c/Users/kahaa/Documents/Github/RDNT/website/* 477grp18@shay.ecn.purdue.edu:~/web/