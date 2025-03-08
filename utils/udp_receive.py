import socket
import utils

# 服务器地址和端口
SERVER_HOST = '172.20.10.4'  # 监听所有可用的接口
SERVER_PORT = 13333

PATH = '../实验数据/NLOS实验/'
FILE_NAME = 'NLOS_full_metal'

# 基站数量
ANCHOR_NUM = 4

# 存储客户端数据的缓存
client_data_cache = {}

def start_server():
    # 创建UDP套接字
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 绑定服务器地址和端口
    server_socket.bind((SERVER_HOST, SERVER_PORT))
    print("[Server] Listening on {}:{}".format(SERVER_HOST, SERVER_PORT))

    try:
        while True:
            # 接收来自客户端的数据和地址信息
            data, client_address = server_socket.recvfrom(8192)
            if client_address[0] not in client_data_cache.keys():
                client_data_cache[client_address[0]] = b''
            else:
                client_data_cache[client_address[0]] += data

            print("[Received] From {}".format(client_address[0]))

    except KeyboardInterrupt:
        print("[Server] Server shutting down...")
    finally:
        # 关闭服务器套接字
        server_socket.close()

        print("[File system] Save data into files")

        # 将缓存数据保存至本地文件
        for key in client_data_cache.keys():
            with open(PATH+FILE_NAME+'_'+key.split('.')[-1]+'.txt', 'wb') as file:
                # print(client_data_cache[key])
                file.write(client_data_cache[key])
                # print(client_data_cache[key].hex('-'))
                file.close()

                utils.byte2txt(byteData=client_data_cache[key], path=PATH, name=FILE_NAME+'_'+key.split('.')[-1], anchor_num=ANCHOR_NUM)
                utils.seperate(path=PATH, name=FILE_NAME+'_'+key.split('.')[-1], tag_num=1, anchor_num=ANCHOR_NUM)

        print("[File system] All data saved")

if __name__ == "__main__":
    start_server()