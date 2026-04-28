HOW TO CONFIGURE MATLAB FOR PYTHON

MATLAB Engine for Python - Setup Guide
======================================
For use with ParaPy + PyCharm (uv-managed venv) on Windows
Tested with: MATLAB R2025a, Python 3.11, Windows 11


PREREQUISITES
-------------
- MATLAB R2025a installed
- PyCharm with a uv-managed project venv
- Python 3.11 installed at:
  C:\Users\<you>\AppData\Local\Programs\Python\Python311\python.exe


STEP 1: Copy the MATLAB engine installer to a user folder
----------------------------------------------------------
Run in PyCharm terminal:

  xcopy "C:\Program Files\MATLAB\R2025a\extern\engines\python" "C:\Users\<you>\matlab_engine_install" /E /I

(This avoids permission issues when building from C:\Program Files)


STEP 2: Point MATLAB to your Python version
--------------------------------------------
Open MATLAB and run:

  pyenv(Version="C:\Users\<you>\AppData\Local\Programs\Python\Python311\python.exe")

Verify with:

  pyenv

Should show Version: "3.11"


STEP 3: Install the MATLAB engine into your venv
-------------------------------------------------
Open PowerShell as Administrator (right-click -> Run as administrator) and run:

First, ensure pip is available in the venv:

  & "C:\Users\<you>\<project-path>\.venv\Scripts\python.exe" -m ensurepip

Then install the engine:

  & "C:\Users\<you>\<project-path>\.venv\Scripts\python.exe" -m pip install "C:\Program Files\MATLAB\R2025a\extern\engines\python"


STEP 4: Verify the installation
--------------------------------
In PyCharm terminal run:

  python -c "import matlab.engine; print('MATLAB engine working!')"


NOTES FOR TEAMMATES
--------------------
- Every teammate needs MATLAB R2025a installed on their own machine
- Every teammate must follow steps 1-4 above individually
- Do NOT add matlabengine to requirements.txt — it will break for anyone without MATLAB
- Add a note to your README pointing teammates to this guide instead


TROUBLESHOOTING
---------------
- "Access Denied" during install     -> Make sure you are running PowerShell as Administrator
- "No module named pip"              -> Run the ensurepip step (Step 3) first
- "Corrupted installation" error     -> Install from the original C:\Program Files path, not the copied folder
- MATLAB shows wrong Python version  -> Restart MATLAB fully, then rerun the pyenv() command
- ParaPy license breaks              -> Run "uv sync" in the terminal to restore the venv, then redo Step 3