"""Paper-inspired single-layer, unidirectional LSTM with external output dropout."""
import torch
from torch import nn


class LSTMRegressor(nn.Module):
    def __init__(self, input_size=2, hidden_size=64, dropout=0.3):
        super().__init__()
        self.recurrent = nn.LSTM(input_size, hidden_size, num_layers=1,
                                 batch_first=True, bidirectional=False, dropout=0.0)
        self.dropout = nn.Dropout(dropout)
        self.output = nn.Linear(hidden_size, 1)

    def forward(self, x):
        sequence, _ = self.recurrent(x)  # Default zero state for each independent window.
        return self.output(self.dropout(sequence[:, -1, :]))

from pathlib import Path
import json,hashlib,argparse
import numpy as np
import pandas as pd
import torch
from scipy.io import savemat

def predict(cycle, refresh=False):
 root=Path(__file__).resolve().parent
 folder=root/'ML_Assets/LSTM'
 meta=json.loads((folder/'metadata.json').read_text());sc=json.loads((folder/'scaler.json').read_text())
 for name,key in [('weights.pt','weights_sha256'),('scaler.json','scaler_sha256')]:
  assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==meta[key]
 spec=meta['config']['architectures']['LSTM'];model=LSTMRegressor(spec['input_size'],spec['hidden_size'],spec['dropout'])
 assert sum(p.numel() for p in model.parameters())==17473
 model.load_state_dict(torch.load(folder/'weights.pt',map_location='cpu',weights_only=True));model.eval()
 frame=pd.read_csv(root/f'Data/{cycle}_processed.csv',float_precision='round_trip');x=frame[['speed_mps','acceleration_mps2']].to_numpy()
 assert sc['feature_columns']==['speed_mps','acceleration_mps2']
 idx=np.maximum(0,np.arange(len(x))[:,None]+np.arange(-9,1)[None,:]);w=((x[idx]-np.array(sc['feature_mean']))/np.array(sc['feature_scale'])).astype(np.float32)
 torch.set_num_threads(1)
 with torch.inference_mode():y=np.concatenate([model(torch.from_numpy(w[k:k+256])).numpy().reshape(-1) for k in range(0,len(w),256)])
 y=y.astype(np.float64)*sc['target_scale']+sc['target_mean']
 saved=root/f'Data/ML_Predictions/LSTM_{cycle}_prediction.csv';ref=pd.read_csv(saved,float_precision='round_trip').raw_prediction_A.to_numpy()
 error=float(np.max(np.abs(y-ref)));assert error<=1e-6,error
 print('LSTM',cycle,'shape',w.shape,'max error A',error)
 if refresh:
  target=root/'Results/Refreshed_Predictions';target.mkdir(exist_ok=True)
  savemat(target/f'LSTM_{cycle}_prediction.mat',{'time_s':frame.time_s.to_numpy()[:,None],'raw_prediction_A':y[:,None],'speed_mps':x[:,0,None],'acceleration_mps2':x[:,1,None],'output_unit':'A'})
 return y
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--refresh',action='store_true');args=parser.parse_args()
 for c in ['EUDC','IM240']:predict(c,args.refresh)
