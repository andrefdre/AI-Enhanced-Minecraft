import torch
import torch.nn as nn
import numpy as np

class KeysPressesLoss(nn.Module):
    def __init__(self):
        super(KeysPressesLoss, self).__init__()
        self.mse_loss = nn.MSELoss()

    def forward(self, y_pred, y_true):
        """
        :param pred: N x 7
        :param targ: N x 7
        :return: 
        """
        batch_size , tensor_size = y_pred.shape

        loss_batch = torch.tensor([])
        loss_batch = loss_batch.to('cuda')

        for j in range(batch_size):
            loss = torch.tensor([])
            loss = loss.to('cuda')
            for i in range(tensor_size):
                if i < 10:
                    print( y_pred[j][i] , y_true[j][i])
                    loss_index = - (y_true[j][i] * torch.log(y_pred[j][i]) + (1 - y_true[j][i]) * torch.log(1 - y_pred[j][i]))
                    print( y_true[j][i] , torch.log(y_pred[j][i]) , (1 - y_true[j][i]) , torch.log(1 - y_pred[j][i]) )
                    loss_index =loss_index.unsqueeze(0)
                    print(loss_index)
                    loss= torch.cat((loss,loss_index))
                else:
                    loss_index = self.mse_loss(y_pred[j][i], y_true[j][i])
                    loss_index =loss_index.unsqueeze(0)
                    loss= torch.cat((loss,loss_index) )

            loss_batch = torch.cat([loss_batch, torch.mean(loss, dim=-1)], dim=-1)

        print (loss_batch)
            
        return loss_batch

