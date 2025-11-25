import socket
import struct
import json
import numpy as np
from PIL import Image
import io
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--host', default='localhost', help='Server host')
parser.add_argument('--port', type=int, default=5555, help='Server port')
parser.add_argument('--color_path', required=True, help='Path to color image')
parser.add_argument('--depth_path', required=True, help='Path to depth image')
parser.add_argument('--mask_path', default=None, help='Path to mask image (optional)')
cfgs = parser.parse_args()


def send_image_data(sock, color_path, depth_path, mask_path=None):
    """
    发送图像数据到服务器
    协议格式：
    1. color_size (4 bytes, int)
    2. color_data (color_size bytes)
    3. depth_size (4 bytes, int)
    4. depth_data (depth_size bytes)
    5. mask_size (4 bytes, int)
    6. mask_data (mask_size bytes)
    """
    try:
        # 读取并编码color图像
        color_img = Image.open(color_path)
        color_buffer = io.BytesIO()
        color_img.save(color_buffer, format='PNG')
        color_data = color_buffer.getvalue()
        
        # 发送color图像大小和数据
        sock.sendall(struct.pack('!I', len(color_data)))
        sock.sendall(color_data)
        print(f"Sent color image: {len(color_data)} bytes")
        
        # 读取并编码depth图像
        depth_img = Image.open(depth_path)
        depth_buffer = io.BytesIO()
        depth_img.save(depth_buffer, format='PNG')
        depth_data = depth_buffer.getvalue()
        
        # 发送depth图像大小和数据
        sock.sendall(struct.pack('!I', len(depth_data)))
        sock.sendall(depth_data)
        print(f"Sent depth image: {len(depth_data)} bytes")
        
        # 发送mask
        if mask_path:
            mask_img = Image.open(mask_path)
            mask_buffer = io.BytesIO()
            mask_img.save(mask_buffer, format='PNG')
            mask_data = mask_buffer.getvalue()
            
            sock.sendall(struct.pack('!I', len(mask_data)))
            sock.sendall(mask_data)
            print(f"Sent mask: {len(mask_data)} bytes")
        else:
            # 发送空mask
            sock.sendall(struct.pack('!I', 0))
            print("No mask sent")
        
        return True
        
    except Exception as e:
        print(f"Error sending data: {e}")
        return False


def receive_results(sock):
    """接收检测结果"""
    try:
        # 接收结果大小
        size_data = sock.recv(4)
        if not size_data:
            return None
        result_size = struct.unpack('!I', size_data)[0]
        print(f"Receiving results: {result_size} bytes")
        
        # 接收结果数据
        result_data = b''
        while len(result_data) < result_size:
            packet = sock.recv(result_size - len(result_data))
            if not packet:
                return None
            result_data += packet
        
        # 解析JSON结果
        result = json.loads(result_data.decode('utf-8'))
        return result
        
    except Exception as e:
        print(f"Error receiving results: {e}")
        return None


def main():
    # 连接服务器
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        print(f"Connecting to {cfgs.host}:{cfgs.port}...")
        client_socket.connect((cfgs.host, cfgs.port))
        print("Connected!")
        
        # 发送图像数据
        if not send_image_data(client_socket, cfgs.color_path, cfgs.depth_path, cfgs.mask_path):
            print("Failed to send data")
            return
        
        print("\nWaiting for results...")
        
        # 接收结果
        result = receive_results(client_socket)
        
        if result:
            print("\n" + "="*50)
            print("DETECTION RESULTS")
            print("="*50)
            print(f"Success: {result['success']}")
            print(f"Number of grasps: {result['num_grasps']}")
            
            if result['success'] and result['num_grasps'] > 0:
                print("\nTop Grasps:")
                for i, grasp in enumerate(result['grasps'][:5]):  # 显示前5个
                    print(f"\nGrasp {i+1}:")
                    print(f"  Score: {grasp['score']:.4f}")
                    print(f"  Width: {grasp['width']:.4f}")
                    print(f"  Height: {grasp['height']:.4f}")
                    print(f"  Depth: {grasp['depth']:.4f}")
                    print(f"  Translation: {grasp['translation']}")
                    print(f"  Rotation Matrix:")
                    rot_mat = np.array(grasp['rotation_matrix'])
                    for row in rot_mat:
                        print(f"    {row}")
                
                # 保存完整结果到文件
                output_file = 'grasp_results.json'
                with open(output_file, 'w') as f:
                    json.dump(result, f, indent=2)
                print(f"\nFull results saved to {output_file}")
            else:
                print("\nNo grasps detected")
            
            print("="*50)
        else:
            print("Failed to receive results")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        client_socket.close()
        print("\nConnection closed")


if __name__ == '__main__':
    main()
