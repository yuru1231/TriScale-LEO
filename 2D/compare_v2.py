import json, numpy as np, csv, os

CPP_CSV  = r'c:\Users\wenj\Desktop\TriScale-LEO\2D\projection\output\v2\channel_results.csv'
PY_BASE  = r'c:\Users\wenj\Desktop\TriScale-LEO\2D\Multi-Beam LEO Communication Satellite Simulation Framework\results'

cpp_snr, cpp_sinr, cpp_bg, cpp_bi, cpp_pl = [], [], [], [], []
with open(CPP_CSV) as f:
    for row in csv.DictReader(f):
        cpp_snr.append(float(row['snr_dB']))
        cpp_sinr.append(float(row['sinr_dB']))
        cpp_bg.append(float(row['beam_gain_dB']))
        cpp_bi.append(int(row['beam_id']))
        cpp_pl.append(float(row['path_loss_dB']))

cpp_snr  = np.array(cpp_snr)
cpp_sinr = np.array(cpp_sinr)
cpp_bg   = np.array(cpp_bg)
cpp_bi   = np.array(cpp_bi)
cpp_pl   = np.array(cpp_pl)

with open(os.path.join(PY_BASE, 'macro_with_Rician38537_100km.json')) as f:
    d = json.load(f)
py_snr  = np.array(d['snr'])
py_sinr = np.array(d['sinr'])
py_bg   = np.array(d['center_beam_gain_dB'])
py_bi   = np.array(d['beam_index'])

sep = '=' * 62
print(sep)
print('Metric                    C++ v2       Python Rician')
print('-' * 62)
print('n_user               {:>10}    {:>10}'.format(len(cpp_snr), len(py_snr)))
print('elevation (deg)      {:>10}    {:>10}'.format('89.997', '90.000'))
print()
print('path_loss mean(dB)   {:>10.3f}           N/A'.format(cpp_pl.mean()))
print('beam_gain max(dB)    {:>10.3f}    {:>10.3f}'.format(cpp_bg.max(), py_bg.max()))
print('beam_gain mean(dB)   {:>10.3f}    {:>10.3f}'.format(cpp_bg.mean(), py_bg.mean()))
print('beam_gain min(dB)    {:>10.3f}    {:>10.3f}'.format(cpp_bg.min(), py_bg.min()))
print()
print('SNR  max(dB)         {:>10.3f}    {:>10.3f}'.format(cpp_snr.max(), py_snr.max()))
print('SNR  mean(dB)        {:>10.3f}    {:>10.3f}'.format(cpp_snr.mean(), py_snr.mean()))
print('SNR  min(dB)         {:>10.3f}    {:>10.3f}'.format(cpp_snr.min(), py_snr.min()))
print()
print('SINR max(dB)         {:>10.3f}    {:>10.3f}'.format(cpp_sinr.max(), py_sinr.max()))
print('SINR mean(dB)        {:>10.3f}    {:>10.3f}'.format(cpp_sinr.mean(), py_sinr.mean()))
print('SINR min(dB)         {:>10.3f}    {:>10.3f}'.format(cpp_sinr.min(), py_sinr.min()))
print()
print('beam=9 users         {:>10}    {:>10}'.format(int(np.sum(cpp_bi==9)), int(np.sum(py_bi==9))))
print(sep)

print()
print('[C++ v2] beam distribution:')
u, c = np.unique(cpp_bi, return_counts=True)
for b, cnt in zip(u, c):
    print('  beam {:2d}: {:7d} users  ({:.1f}%)'.format(b, cnt, cnt/len(cpp_bi)*100))

print()
print('[Python] beam distribution:')
u, c = np.unique(py_bi, return_counts=True)
for b, cnt in zip(u, c):
    print('  beam {:2d}: {:7d} users  ({:.1f}%)'.format(b, cnt, cnt/len(py_bi)*100))
