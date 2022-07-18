function info = sim_getfile_info(filename)

    [path, name, ext] = fileparts(filename);
    
    fields = regexp(name, '\.', 'split');

    info.subject    = fields{1};
    info.date       = fields{2};
    info.time       = fields{3};
    info.modality   = fields{4};
    info.navigation = fields{5};
    info.control    = fields{6};

    info.extension = ext;
    info.filepath  = path;


end