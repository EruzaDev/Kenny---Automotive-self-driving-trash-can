# Selected guarded model

This is the seed 3 guarded checkpoint from the reviewed 21 × 21 m mixed-stage training run, selected for the current map demonstration. The selection follows the project technical defense guide. It is the guarded-validation checkpoint, rather than the final training snapshot.

The archive includes the PPO policy and optimizer. Keep `config.json` and `contract.json` beside it for the project evaluation, replay, and demo loaders. `metadata.json` records training provenance, and `requirements.txt` records the training dependencies.

For the current map demonstration, use the shielded route recovery and marker sweep controller with a 1,800-step budget. Controller settings are separate from the trained checkpoint. This model has been evaluated in simulation; the selection does not establish physical-world reliability.

From the repository root, load the checkpoint with:

```sh
python -m kenny_rl.demo --model runs/server-mixed-21x21-reviewed/seed_3/best_guarded.zip --stage full --seed 700
```

Verify the included files from this directory with `sha256sum -c SHA256SUMS`.
