# CS223a: Introduction to Robotics

## Installation

1. This code officially supports Mac and Ubuntu, although it is possible to run it on Windows using the Windows Subsystem for Linux.

   ### Mac
   
   1. Install Brew (https://brew.sh)
   2. Run ```xcode-select --install```in the terminal to install MacOS command line tools
   
   ### Ubuntu
   
   1. Go to step 2!
   
   ### Windows
   
   1. Install Windows Subsystem for Linux (https://docs.microsoft.com/en-us/windows/wsl/install-win10)
   2. Launch an Ubuntu Bash terminal

2. Run the setup script in the top directory from the terminal. This will download the required external libraries and compile them.

   ```
   ./setup.sh
   ```
   
   The setup script simply runs the commands you would type in the terminal normally. You can look at the script and follow the steps manually if something fails along the way.

3. Install Python 3, pip, and virtualenv. You can do this automatically with the following script, or use your own preferred method.

   ### Automatic

   ```
   ./setup_python.sh
   ```

   ### Manual

   ```
   pip install virtualenv           # Install virtualenv if you don't already have it
   virtualenv env                   # Create a new virtual environment in the folder "./env"
   source env/bin/activate          # Activate the virtual environment
   pip install -r requirements.txt  # Install the required Python packages in the virtual environment
   ```

## Usage

1. Activate the Python virtual environment. You will have to do this every time you open a new terminal.

   ```
   source env/bin/activate
   ```
  
2. The code you need to implement will be in the ```cs223a``` folder.

3. To run the visualizer, first start the Redis server. On some OSes, the server is executed automatically as a background service on system startup, so this step may be unnecessary.

   ```
   redis-server
   ```

4. Start the visualizer web server (in the virtual environment).

   ```
   ./visualizer.py
   ```

5. Navigate to the following URL in a web browser.

   ```
   localhost:8000
   ```
