#!/bin/bash

# git pull repository
git pull

# Store the password in a variable
password="=yXEnU%a"

# copy file to server using sshpass
sshpass -p $password scp /mnt/c/Users/kahaa/Documents/Github/RDNT/website/* ece477grp18@shay.ecn.purdue.edu:/test/
