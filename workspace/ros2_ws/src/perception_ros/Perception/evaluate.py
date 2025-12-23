import torch
import cv2
import numpy as np

from Perception.utils.utils import find_edge_channel


def preprocess_image(img):
    """
    Convert BGR image to 3-channel UNet input:
    [gray, edges, edges_inv]
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges, edges_inv = find_edge_channel(img)

    output = np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)
    output[:, :, 0] = gray
    output[:, :, 1] = edges
    output[:, :, 2] = edges_inv

    tensor = torch.from_numpy(output).float() / 255.0
    tensor = tensor.permute(2, 0, 1).unsqueeze(0)  # BCHW
    return tensor


def evaluate(model, frame):
    """
    Run lane segmentation on a single image.

    Args:
        model: Loaded UNet model (already in eval mode)
        frame: OpenCV BGR image

    Returns:
        Binary mask (H, W) with values {0,1}
    """
    with torch.no_grad():
        tensor = preprocess_image(frame)
        pred = model(tensor)
        pred = torch.sigmoid(pred)[0, 0].cpu().numpy()

    mask = (pred > 0.5).astype(np.uint8)
    return mask
