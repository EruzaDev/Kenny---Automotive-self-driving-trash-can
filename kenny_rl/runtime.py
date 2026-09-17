"""Fail early instead of silently falling back from a requested GPU to CPU."""


def check_device(device):
    import torch
    if device == "cpu":
        return {"device": "cpu", "name": "CPU"}
    if not device.startswith("cuda:") or not device[5:].isdigit():
        raise ValueError("Device must be cpu or cuda:N (index within CUDA_VISIBLE_DEVICES)")
    index = int(device[5:])
    if not torch.cuda.is_available() or index >= torch.cuda.device_count():
        raise ValueError(f"{device} unavailable; check GPU allocation, driver, CUDA_VISIBLE_DEVICES and CUDA PyTorch")
    # Exercise the runtime as well as enumeration before creating run artifacts.
    value = torch.ones(1, device=device) + 1
    if value.item() != 2:
        raise ValueError(f"CUDA computation failed on {device}")
    return {"device": device, "name": torch.cuda.get_device_name(index),
            "cuda_runtime": torch.version.cuda}
