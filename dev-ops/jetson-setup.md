# Jetson Setup

This tutorial teaches you on how to setup a jetson for remote developers for them to connect


1) Open ssh config and ssd config using the following lines

```
vim ./.ssh/config
vim ./.sshd/config
```

2) Comment in authorized keys and password authentication and comment OUT allow groups
3) Navigate to system using the following command


```
cd etc/systemd/system
```

4) Run the following command to get the service

```
wget www.waltmckelvie.com/files/autossh-tunnel.service
```

5) Verify this installation worked by running ls | grep autossh-tunnel.service

6) Open this service (using vim) and modify the last two numbers of the jetson appropriately (00, 01, 02, 03, etc)

```
vim autossh-tunnel.service
```

7) In this next step, you must generate the keys to connect to the server please cd back to the home directory and run the following command

```
cd ~/.ssh
ssh-keygen
```

8) Find and copy the contents of the rsa.pub file using the following command and email it to Neil to add to the server
 ```
 cat rsa.pub
 ```
 
9) Before running anything make sure autossh is installed by using the following command

```
sudo apt get install autossh
```

10) The user stuff

11) Once you are verified as the root user run the following command and it should work

```
sudo systemctl start autossh-tunnel.service
```

In order to make sure the system is running, verify it by connecting to it from the client machine. Here are the following steps on connecting from a client machine to a jetson. Here is the link on how to do that https://github.com/columbia-university-robotics/how-to/blob/main/connect-jetson-servers.md

