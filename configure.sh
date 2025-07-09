git submodule update --init --recursive
cd mavlink
mkdir generated
python -m venv venv
PYTH=venv/bin/python
$PYTH --version
$PYTH -m pip install -r pymavlink/requirements.txt
$PYTH -m pip install --upgrade pip setuptools wheel
$PYTH -m pip install pymavlink future

# Change the message_definition you are using at the end if you need features that are not present in common.
# Or if you are using a custom dialect.
$PYTH -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=./generated ./message_definitions/v1.0/common.xml

cp -r generated/ ../mavlink_headers
cd ../
mkdir build
cd build
cmake ../
cmake --build .
