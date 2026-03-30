import torch
import torch.nn as nn


class Network(nn.Module):
    def __init__(self, n_input=18, n_layer_1=128, n_layer_2=64, n_output=2):
        super(Network, self).__init__()
        self.fc1 = nn.Linear(n_input, n_layer_1)
        self.fc2 = nn.Linear(n_layer_1, n_layer_2)
        self.fc3 = nn.Linear(n_layer_2, n_output)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

    def load_saved_policy(self, state_dict):
        self.fc1.weight.data = state_dict['p_fc1.weight']
        self.fc1.bias.data = state_dict['p_fc1.bias']
        self.fc2.weight.data = state_dict['p_fc2.weight']
        self.fc2.bias.data = state_dict['p_fc2.bias']
        self.fc3.weight.data = state_dict['p_fc3.weight']
        self.fc3.bias.data = state_dict['p_fc3.bias']
