# To get things runnig 

```
cd src/
conda-develop .
```
 
## Installing MuJoCo:
(https://github.com/google-deepmind/mujoco)
`pip install mujoco-py`

## Conda env in:
`conda env create -f env.yaml`

## Then:

`python rl_rp.py`
or
`python parallel_rl_rp.py`

`python try_policy.py --policy pid`

Run the server:
`mlflow server`

To see the server:
127.0.0.1:5000


