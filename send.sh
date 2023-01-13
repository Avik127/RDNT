#!/bin/bash

# git pull repository
git pull

# Store the password in a variable
password="=yXEnU%a"

# copy file to server using sshpass
# chnage the source path /mnt/c/Users/kahaa/Documents/Github/RDNT/website/* according to your soruce file path
sshpass -p =yXEnU%a scp -r /mnt/c/Users/kahaa/Documents/Github/RDNT/website/* 477grp18@shay.ecn.purdue.edu:~/web/
