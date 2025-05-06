function filename=generateFootVTP(heel, toe, ankle, filename)
% generateFootVTP Generates a .vtp file for a COM‐centered foot wedge.
%   generateFootVTP(filename) writes out a VTP file that represents a
%   foot wedge (triangular prism) whose design geometry is defined by
%   the corner locations of the triangle
    %% Define the design parameters (in meters)
    % Given "ankle height" is used for the extrusion thickness.
    folderPath = '/Applications/OpenSim 4.5/OpenSim.app/Contents/Resources/opensim/Geometry/';

    ankle_height = ankle(2);
    z_bottom = -ankle_height/2;
    z_top    =  ankle_height/2;
    
    %% Compute the 2D centroid of the design triangle
    centroid = (heel + toe + ankle) / 3;  % (x,y) centroid
    % Display the computed centroid (optional)
    % fprintf('Computed centroid in xy: (%.5f, %.5f)\n', centroid(1), centroid(2));
    
    %% Shift the design points so that the centroid is at (0,0)
    heel_shifted  = heel  - centroid;
    toe_shifted   = toe   - centroid;
    ankle_shifted = ankle - centroid;
    
    %% Define the 6 points (vertices) for the two parallel triangles
    % Bottom triangle (z = z_bottom)
    p0 = [heel,  z_bottom];   % Heel (bottom)
    p1 = [toe,   z_bottom];    % Toe (bottom)
    p2 = [ankle, z_bottom];    % Ankle (bottom)
    
    % Top triangle (z = z_top)
    p3 = [heel,  z_top];
    p4 = [toe,   z_top];
    p5 = [ankle, z_top];
    
    pts = [p0; p1; p2; p3; p4; p5];  % 6x3 matrix

    %pts(:,[1,3]) = -pts(:,[1,3]); % Mirror
    
    %% Define the connectivity for the 5 polygon faces.
    % The ordering of vertices for each face:
    %   Face 1 (bottom triangle):  [0 1 2]
    %   Face 2 (top triangle):     [3 4 5]
    %   Face 3 (side quad 1):      [0 1 4 3]
    %   Face 4 (side quad 2):      [1 2 5 4]
    %   Face 5 (side quad 3):      [2 0 3 5]
    % (Indices in MATLAB start at 1, but VTK indices must start at 0.)
    connectivityList = { [0 1 2], [3 4 5], [0 1 4 3], [1 2 5 4], [2 0 3 5] };
    offsets = [3, 6, 10, 14, 18];  % Cumulative vertex counts.
    
    %% Write the VTP file
    fullFilePath = [folderPath, filename];
    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not open file %s for writing.', filename);
    end
    
    % Write XML header and opening tags
    fprintf(fid, '<?xml version="1.0"?>\n');
    fprintf(fid, '<VTKFile type="PolyData" version="1.0" byte_order="LittleEndian">\n');
    fprintf(fid, '  <PolyData>\n');
    fprintf(fid, '    <Piece NumberOfPoints="6" NumberOfPolys="5">\n');
    fprintf(fid, '      <PointData/>\n');
    fprintf(fid, '      <CellData/>\n');
    
    % Write the points section
    fprintf(fid, '      <Points>\n');
    fprintf(fid, '        <DataArray type="Float32" NumberOfComponents="3" format="ascii">\n');
    for i = 1:size(pts,1)
        fprintf(fid, '          %.5f %.5f %.5f\n', pts(i,1), pts(i,2), pts(i,3));
    end
    fprintf(fid, '        </DataArray>\n');
    fprintf(fid, '      </Points>\n');
    
    % Write the Polys section: connectivity and offsets arrays.
    fprintf(fid, '      <Polys>\n');
    
    % Write connectivity array.
    fprintf(fid, '        <DataArray type="Int32" Name="connectivity" format="ascii">\n');
    for i = 1:length(connectivityList)
        poly = connectivityList{i};
        % Write the numbers with a leading indent
        fprintf(fid, '          ');
        fprintf(fid, '%d ', poly);
        fprintf(fid, '\n');
    end
    fprintf(fid, '        </DataArray>\n');
    
    % Write offsets array.
    fprintf(fid, '        <DataArray type="Int32" Name="offsets" format="ascii">\n');
    for i = 1:length(offsets)
        fprintf(fid, '          %d\n', offsets(i));
    end
    fprintf(fid, '        </DataArray>\n');
    
    % Close the tags.
    fprintf(fid, '      </Polys>\n');
    fprintf(fid, '    </Piece>\n');
    fprintf(fid, '  </PolyData>\n');
    fprintf(fid, '</VTKFile>\n');
    
    fclose(fid);
    fprintf('VTP file written to %s at %s\n', filename, folderPath);
end
