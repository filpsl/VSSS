# Python VSS Client

This project provides a sample client to interact with IEEE Very Small Size Soccer simulators as [TraveSim](https://github.com/futebol-mini/travesim) and [FIRASim](https://github.com/futebol-mini/FIRASim)

## Virtual environment

We recomend the use of a python virtual environment. It may be created with

```bash
python -m venv venv
```

Then, activate it with

```bash
source venv/bin/activate
```

## Requirements

Install the client requirements with

```bash
pip install -r requirements.txt
```

Its only requirements are Google's protobuf and VSS [proto definitions](https://github.com/futebol-mini/VSSProto.py)
