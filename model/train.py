import argparse
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader, random_split
from tqdm import tqdm
from model import Skeleton2Mesh
from torch.utils.tensorboard import SummaryWriter
from datetime import datetime
import math 

# ==========================================
# [USER CONFIG]
# ==========================================
USE_RANGE_PENALTY = True   
PENALTY_WEIGHT    = 1.0    

LIMIT_MIN = [-15.0, -15.0, -15.0] 
LIMIT_MAX = [ 25.0,  15.0,  18.0] 

# [NOISE CONFIG]
USE_NOISE    = True     
NOISE_DEGREE = 5.0      # Maximum 5 degrees rotation noise

# ==========================================
# [NEW] Helper Function: Rotation Noise
# ==========================================
def apply_rotation_noise(tensor, mode, degree=5.0):
    """
    입력 텐서의 '마지막' N개 요소(회전 정보)에만 노이즈를 적용합니다.
    - Euler: 마지막 3개 (Additive Noise)
    - Quat : 마지막 4개 (Geometric Noise, xyzw)
    - Rot6D: 마지막 6개 (Geometric Noise, Row-Major Input)
    """
    if not USE_NOISE or degree <= 0:
        return tensor

    # 1. Set rotation dimension according to mode
    if mode == "euler":
        rot_dim = 3
    elif mode == "quat":
        rot_dim = 4
    elif mode == "rot6d":
        rot_dim = 6
    else:
        return tensor

    # 2. Separate data (front: fixed / back: noise target)
    static_part = tensor[:, :-rot_dim]
    target_part = tensor[:, -rot_dim:] 
    
    batch_size = target_part.shape[0]
    device = target_part.device
    max_rad = degree * (math.pi / 180.0) # Degree -> Radian conversion

    # ---------------------------------------------
    # Case 1: Euler (Last 3)
    # ---------------------------------------------
    if mode == "euler":
        # Euler: simple value addition
        noise = (torch.rand_like(target_part) * 2 - 1) * max_rad
        target_noised = target_part + noise
        target_noised = (target_noised + math.pi) % (2 * math.pi) - math.pi

    # ---------------------------------------------
    # Case 2: Quaternion (Last 4) - xyzw Order
    # ---------------------------------------------
    elif mode == "quat":
        # target_part: [Batch, 4] -> (x, y, z, w)
        
        # Generate random axis & angle
        rand_axis = torch.randn(batch_size, 3, device=device)
        rand_axis = rand_axis / (torch.norm(rand_axis, dim=1, keepdim=True) + 1e-8)
        rand_angle = (torch.rand(batch_size, 1, device=device) * 2 - 1) * max_rad
        theta_half = rand_angle / 2

        # Noise Quaternion (xyzw order)
        noise_xyz = torch.sin(theta_half) * rand_axis
        noise_w   = torch.cos(theta_half)
        
        # Hamilton Product (q_noise * q_target)
        # Multiplication formula for xyzw order
        v1, w1 = noise_xyz, noise_w
        v2, w2 = target_part[:, :3], target_part[:, 3:] 

        new_w = w1 * w2 - torch.sum(v1 * v2, dim=1, keepdim=True)
        new_v = w1 * v2 + w2 * v1 + torch.cross(v1, v2, dim=1)

        target_noised = torch.cat([new_v, new_w], dim=1)
        # Unit Quaternion normalization
        target_noised = target_noised / (torch.norm(target_noised, dim=1, keepdim=True) + 1e-8)

    # ---------------------------------------------
    # Case 3: 6D Rotation (Last 6) - Row Major Match
    # ---------------------------------------------
    elif mode == "rot6d":
        # 1. Generate random rotation matrix (3x3) - Rodrigues Formula
        rand_axis = torch.randn(batch_size, 3, device=device)
        rand_axis = rand_axis / (torch.norm(rand_axis, dim=1, keepdim=True) + 1e-8)
        rand_angle = (torch.rand(batch_size, 1, device=device) * 2 - 1) * max_rad
        
        z = torch.zeros_like(rand_axis[:, 0])
        k_mat = torch.stack([
             z, -rand_axis[:, 2],  rand_axis[:, 1],
             rand_axis[:, 2],  z, -rand_axis[:, 0],
            -rand_axis[:, 1],  rand_axis[:, 0],  z
        ], dim=1).reshape(batch_size, 3, 3)

        theta = rand_angle.unsqueeze(2)
        eye = torch.eye(3, device=device).unsqueeze(0).expand(batch_size, -1, -1)
        
        # R_noise: [Batch, 3, 3] random rotation matrix
        R_noise = eye + torch.sin(theta) * k_mat + (1 - torch.cos(theta)) * torch.matmul(k_mat, k_mat)

        # 2. Apply vector rotation (Row-Major data handling)
        # User data generation method: reshape(..., -1) -> Row Major flattening
        # Format: [r00, r01, r10, r11, r20, r21]
        
        # PyTorch's .view(B, 3, 2) fills in order, so it accurately restores to 3x2 matrix.
        # [[r00, r01],
        #  [r10, r11],
        #  [r20, r21]]
        input_vecs = target_part.view(batch_size, 3, 2)
        
        # 3. Rotation matrix multiplication: R_noise(3x3) * Input(3x2) -> Output(3x2)
        out_vecs = torch.matmul(R_noise, input_vecs)
        
        # 4. Flatten back to original order (Row-Major)
        target_noised = out_vecs.view(batch_size, 6)

    # 3. Combine front part of original with noised back part
    return torch.cat([static_part, target_noised], dim=1)


# =========================
# Argument parser
# =========================
def parse_args():
    parser = argparse.ArgumentParser(description="Train Skeleton2Mesh")

    # Basic configurations
    parser.add_argument("--use_euler", action="store_true", help="Use Euler angle")
    parser.add_argument("--use_quat", action="store_true", help="Use Quaternion")
    parser.add_argument("--use_6d", action="store_true", help="Use 6D Rotation")

    parser.add_argument("--epochs", type=int, default=6000)
    parser.add_argument("--batch_size", type=int, default=1024)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--weight_decay", type=float, default=1e-5)

    return parser.parse_args()


# =========================
# Main
# =========================
def main():
    args = parse_args()

    # ---- Flag Validation ----
    if sum([args.use_euler, args.use_quat, args.use_6d]) > 1:
        raise ValueError("Error: Select only ONE of --use_euler, --use_quat, --use_6d")

    # Name & Mode setup
    if args.use_6d:
        name = "rot6d"
        mode_str = "rot6d"
    elif args.use_euler:
        name = "euler"
        mode_str = "euler"
    elif args.use_quat:
        name = "quat"
        mode_str = "quat"
    else:
        name = "no_euler"
        mode_str = "none"

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    writer = SummaryWriter(log_dir=f"runs/{name}/{run_id}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    print(f"Mode: {name}")

    # =========================
    # [Setup] Penalty Tensors
    # =========================
    if USE_RANGE_PENALTY:
        tensor_min = torch.tensor(LIMIT_MIN, device=device, dtype=torch.float32) * (math.pi / 180.0)
        tensor_max = torch.tensor(LIMIT_MAX, device=device, dtype=torch.float32) * (math.pi / 180.0)

        print("-" * 30)
        print(f"Constraint Active (Weight: {PENALTY_WEIGHT})")
        print(f"Noise Augmentation: {USE_NOISE} (Max {NOISE_DEGREE} deg)")
        print(f"Noise Target: Last N elements (Euler=3, Quat=4, 6D=6)")
        print("-" * 30)
    
    # =========================
    # Fixed configuration
    # =========================
    num_joints     = 20
    num_bones      = 19
    gsd_dim        = 100
    mlp_hidden     = [256, 256]
    pe_freqs_bone  = 5
    pe_freqs_order = 2

    # =========================
    # Dataset
    # =========================
    if args.use_6d:
        dataset_path = 'dataset_new_6d_rad.pt'
    elif args.use_quat:
        dataset_path = 'dataset_new_quat_rad.pt'
    else:
        dataset_path = 'dataset_new_rad.pt'
    
    print(f"Loading dataset from {dataset_path}...")
    loaded_data = torch.load(dataset_path)

    if isinstance(loaded_data, dict):
        X_tensor = loaded_data['inputs']
        y_tensor = loaded_data['labels']
    else:
        X_tensor, y_tensor = loaded_data

    dataset = TensorDataset(X_tensor, y_tensor)
    train_size = int(0.9 * len(dataset))
    val_size   = len(dataset) - train_size
    train_ds, val_ds = random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(
        train_ds, batch_size=args.batch_size, shuffle=True,
        num_workers=2, pin_memory=True, persistent_workers=True
    )

    val_loader = DataLoader(
        val_ds, batch_size=args.batch_size, shuffle=False,
        num_workers=2, pin_memory=True, persistent_workers=True
    )

    # =========================
    # Model / Optimizer
    # =========================
    model = Skeleton2Mesh(
        num_joints=num_joints,
        num_bones=num_bones,
        gsd_dim=gsd_dim,
        mlp_hidden=mlp_hidden,
        pe_freqs_bone=pe_freqs_bone,
        pe_freqs_order=pe_freqs_order,
        use_euler=args.use_euler,
        use_quat=args.use_quat,
        use_6d=args.use_6d 
    ).to(device)

    optimizer = optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    criterion = nn.MSELoss()
    scheduler = optim.lr_scheduler.CosineAnnealingWarmRestarts(
        optimizer, T_0=args.epochs, T_mult=2, eta_min=args.lr
    )

    # =========================
    # Training loop
    # =========================
    best_val_loss = float("inf")
    tqdm_iterator = tqdm(range(1, args.epochs + 1))

    for epoch in tqdm_iterator:

        # ---- Training ----
        model.train()
        train_loss = 0.0
        train_penalty = 0.0

        for x_batch, y_batch in train_loader:
            x_batch = x_batch.to(device)
            y_batch = y_batch.to(device)

            # [NEW] Apply Noise to Input (Last dim elements)
            if USE_NOISE and epoch > 0.5 * args.epochs:
                x_batch = apply_rotation_noise(x_batch, mode=mode_str, degree=NOISE_DEGREE)

            preds = model(x_batch) 
            
            # 1. MSE Loss
            mse_loss = criterion(preds, y_batch)

            # 2. Range Penalty Calculation
            penalty_loss = torch.tensor(0.0, device=device)
            
            if USE_RANGE_PENALTY:
                violation_min = torch.relu(tensor_min - preds) 
                violation_max = torch.relu(preds - tensor_max)
                penalty_loss = torch.mean(violation_min**2 + violation_max**2)

            # 3. Total Loss
            loss = mse_loss + (PENALTY_WEIGHT * penalty_loss)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_loss += loss.item() * x_batch.size(0)
            if USE_RANGE_PENALTY:
                train_penalty += penalty_loss.item() * x_batch.size(0)

        train_loss /= train_size
        train_penalty /= train_size

        # ---- Validation ----
        model.eval()
        val_loss = 0.0

        with torch.no_grad():
            for x_batch, y_batch in val_loader:
                x_batch = x_batch.to(device)
                y_batch = y_batch.to(device)

                preds = model(x_batch)
                mse_loss = criterion(preds, y_batch)
                
                penalty_loss = torch.tensor(0.0, device=device)
                if USE_RANGE_PENALTY:
                    violation_min = torch.relu(tensor_min - preds)
                    violation_max = torch.relu(preds - tensor_max)
                    penalty_loss = torch.mean(violation_min**2 + violation_max**2)
                
                loss = mse_loss + (PENALTY_WEIGHT * penalty_loss)
                val_loss += loss.item() * x_batch.size(0)

        val_loss /= val_size

        # Logging
        desc = f"Epoch {epoch:04d}"
        postfix = {"train_loss": f"{train_loss:.6f}", "val_loss": f"{val_loss:.6f}"}
        
        if USE_RANGE_PENALTY and train_penalty > 1e-8:
            postfix["penalty"] = f"{train_penalty:.6f}"
            
        tqdm_iterator.set_description(desc)
        tqdm_iterator.set_postfix(postfix)

        writer.add_scalar("Loss/train", train_loss, epoch)
        writer.add_scalar("Loss/val", val_loss, epoch)
        if USE_RANGE_PENALTY:
            writer.add_scalar("Loss/penalty_raw", train_penalty, epoch)

        writer.add_scalar("LR", optimizer.param_groups[0]["lr"], epoch)

        if epoch == 1 and USE_RANGE_PENALTY:
            writer.add_text("Config", 
                            f"Min: {LIMIT_MIN}\nMax: {LIMIT_MAX}\nNoise: {USE_NOISE} ({NOISE_DEGREE} deg)")

        # Checkpoint Saving
        if val_loss < best_val_loss and epoch > 0.8 * args.epochs:
            best_val_loss = val_loss
            ckpt = {
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "loss": val_loss,
                "args": vars(args)
            }
            path = f"checkpoint_best_val_loss_{name}.pth"
            torch.save(ckpt, path)
            tqdm.write(f"New best checkpoint saved: {path}")

        scheduler.step()

    final_ckpt = {
        "epoch": epoch,
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "loss": val_loss,
        "args": vars(args)
    }
    torch.save(final_ckpt, f"checkpoint_final_{name}.pth")
    print("Training Finished.")
    
    tqdm_iterator.close()
    writer.close()

if __name__ == "__main__":
    main()