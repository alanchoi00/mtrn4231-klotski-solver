# Common Rosbridge Issue
You may run into an issue while installing rosbridge. 
The most up to date version of rosbridge has changed repositories, so attemting to install it from its old location will cause a 404
To ensure you are installing from the right place run the following commands

```bash
# remove expired keys
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null
sudo rm /usr/share/keyrings/ros.archive-keyring.gpg 2>/dev/null

# get the new key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# update
sudo apt update

```

After this, if all goes well, you should be able to install rosbridge without issues.


