#! /usr/bin/env python3.11
# TODO: Need testing

import functools
import os
from pathlib import Path
from select import select
import sys
import json
import jax.numpy as jp
import jax
import traceback
from protocol.messages import InferenceMsg, InferenceReply, parse_message
from brax.training.types import Policy
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import checkpoint



def _load_policy(checkpoint_dir: Path) -> Policy:
    """Restore a policy from `checkpoint_dir`."""
    make_network = functools.partial(ppo_networks.make_ppo_networks, policy_obs_key = "actor", value_obs_key = "critic")
    return checkpoint.load_policy(str(checkpoint_dir), deterministic=True, network_factory=make_network)

def send(msg):
    sys.stdout.write(json.dumps(msg) + "\n")
    sys.stdout.flush()
    

if len(sys.argv) != 4:
    sys.stderr.write("Usage: python inference_server.py <checkpoint_dir> <observation_shape> <r_fd>\n")
    sys.exit(1)
      
checkpoint_dir = Path(sys.argv[1])
observation_shape = tuple(map(int, sys.argv[2].strip("()").split(",")))
r_fd = int(sys.argv[3])

# Load the policy from the provided path
try:
    inference_fn: Policy = _load_policy(checkpoint_dir)
except Exception as e:
    inference_reply = InferenceReply(status="error", error=str(e), traceback=traceback.format_exc())
    send(inference_reply.to_json())
    sys.exit(1)
    
# JIT compile the inference function and warm-up 
inference_fn = jax.jit(inference_fn)
for _ in range(10):
    # Generate a warmup observation
    warmup_observation = jp.zeros(observation_shape, dtype=jp.float32)
    try:
        inference_fn(warmup_observation)
    except Exception as e:
        inference_reply = InferenceReply(status="error", error=str(e), traceback=traceback.format_exc())
        send(inference_reply.to_json())
        sys.exit(1)


# MAIN LOOP
fds = [sys.stdin, r_fd]
while True:
    ready, _, _ = select(fds, [], [])
    
    if r_fd in ready:
        if os.read(r_fd, 1) == b'':
            sys.stderr.write("Parent process has terminated. Exiting.\n")
            sys.exit(0)

        
    if sys.stdin in ready:
        # --- Server work ---
        msg = sys.stdin.readline().strip()
        try:
            inf_message = parse_message(msg)
            
            if isinstance(inf_message, InferenceMsg):
                obs = jp.array(inf_message.obs, dtype=jp.float32)
                result :jp.ndarray= inference_fn(obs)
                inference_reply = InferenceReply(status="ok", result= result.tolist())
                send(inference_reply.to_json())

            else:
                inference_reply = InferenceReply(status="error", error="Unknown command")
                send(inference_reply.to_json())

        except Exception as e:
            inference_reply = InferenceReply(status="error", error=str(e), traceback=traceback.format_exc())
            send(inference_reply.to_json())