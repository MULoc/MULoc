function device = read_device_dw1000(path, file_name, device_name, anchor_num, device_type)

device.name = device_name;
single_len = 0;
format = '';

if device_type == 1
    device_num = anchor_num;  % tag
    single_len = 8;
    for i = 1:anchor_num
        format = [format, '%s%s%s%s%s%s%s%f'];
    end
    format = [format, '%s'];
elseif device_type == 0
    device_num = anchor_num-1;  % anchor
    single_len = 7;
    for i = 1:anchor_num-1
        format = [format, '%s%s%s%s%s%s%f'];
    end
    format = [format, '%s'];
end

tic
data = table2cell(readtable([path,file_name,'_',device_name,'.txt'],'Format',format));
toc
for i = 1:length(data(:,1))
    for j = 1:device_num

        if device_type == 1
            data{i,(j-1)*single_len+1} = hex2dec(data{i,(j-1)*single_len+1}); % CIR real
            data{i,(j-1)*single_len+2} = hex2dec(data{i,(j-1)*single_len+2}); % CIR imag
            data{i,(j-1)*single_len+3} = hex2dec(data{i,(j-1)*single_len+3}); % phase calib
            data{i,(j-1)*single_len+4} = hex2dec(data{i,(j-1)*single_len+4}); % rx preamble count
            data{i,(j-1)*single_len+5} = hex2dec(data{i,(j-1)*single_len+5}); % max growth cir
            data{i,(j-1)*single_len+6} = hex2dec(data{i,(j-1)*single_len+6}); % rx  timestamp
            data{i,(j-1)*single_len+7} = hex2dec(data{i,(j-1)*single_len+7}); % carrier interger
        elseif device_type == 0
            data{i,(j-1)*single_len+1} = hex2dec(data{i,(j-1)*single_len+1}); % CIR real
            data{i,(j-1)*single_len+2} = hex2dec(data{i,(j-1)*single_len+2}); % CIR imag
            data{i,(j-1)*single_len+3} = hex2dec(data{i,(j-1)*single_len+3}); % phase calib
            data{i,(j-1)*single_len+4} = hex2dec(data{i,(j-1)*single_len+4}); % rx preamble count
            data{i,(j-1)*single_len+5} = hex2dec(data{i,(j-1)*single_len+5}); % max growth cir
            data{i,(j-1)*single_len+6} = hex2dec(data{i,(j-1)*single_len+6}); % rx  timestamp
        end

    end
    data{i,end} = hex2dec(data{i,end});
    
end

data = cell2mat(data);

data_len = length(data(:,1));

device.rx_times = zeros(anchor_num, data_len);
device.poa = zeros(anchor_num, data_len);

device.idx = data(:,end);

for i = 1:device_num
    
    idx = data(1,i*single_len)+1;

    real_part = data(:,(i-1)*single_len+1)';
    imag_part = data(:,(i-1)*single_len+2)';
    
    real_part(real_part>2^15) = real_part(real_part>2^15) - 2^16;
    imag_part(imag_part>2^15) = imag_part(imag_part>2^15) - 2^16;

    device.poa(idx,:) = angle(real_part+1j*imag_part);
    device.abs(idx,:) = abs(real_part+1j*imag_part);
    device.complex(idx,:) = real_part+1j*imag_part;

    device.phase_cali(idx,:) = data(:,(i-1)*single_len+3)';

    c = data(:,(i-1)*single_len+5);
    n = data(:,(i-1)*single_len+4);
    a = 121.74;
    corrFac = 0.72;
    power = 10*log10(c.*(2^17) ./ (n.^2)) - a;
    power(power > -88) = power(power > -88) + (power(power > -88)+88)*corrFac;
    device.rssi(idx,:)  = power;

    device.poa(idx,:) = device.poa(idx,:) - device.phase_cali(idx,:)/128*2*pi;

    if device_type == 1
        device.rx_times(idx,:) = data(:,(i-1)*single_len+6)';
        temp = data(:,(i-1)*single_len+7);
        temp(temp>2^31) = temp(temp>2^31) - 2^32;
        device.cfo(idx,:) = -temp'*(998.4e6/2.0/1024.0/131072.0);
    else
        device.rx_times(idx,:) = data(:,(i-1)*single_len+6)';
    end

end

end