function output = webconverter( folderpath, matlen )
    standalone = ~exist( "folderpath", "var" );
    if standalone
        folderpath = "..\site";
        matlen = 100;
    end
    outputstring = "";
    for file = dir( folderpath )'
        if ~file.isdir
            filepath = fullfile( file.folder, file.name );
            [ ~, name, extension ] = fileparts( filepath );
            fo = fopen( filepath );
            bytedata = fread( fo );
            fclose( fo );
            clc;
            outputstring = append( outputstring, "const char ", name, "[] =" );
            if strcmp( extension, ".png" )
                arrlen = size( bytedata, 2 );
                lines = char( append( "\x", string( dec2hex( bytedata ) ) ) );
                lines = reshape( lines', [ size( lines, 2 ) * size( lines, 1 ), 1 ] )';
                lines = [ lines, char( zeros( 1, matlen - mod( length( lines ), matlen ) ) ) ];
                lines = replace( string( reshape( lines, [ matlen, length( lines ) / matlen ] )' ), char( 0 ), '' );
                for line = lines'
                    outputstring = append( outputstring, newline, "  ", '"', line, '"' );
                end
            else
                lines = join( strip( replace( split( char( bytedata )', newline ), char( 13 ), '' ) ), '' );
                lines = lines{1};
                arrlen = size( lines, 2 );
                lines = [ lines, char( zeros( 1, matlen - mod( length( lines ), matlen ) ) ) ];
                lines = replace( replace( replace( string( reshape( lines, [ matlen, length( lines ) / matlen ] )' ), char( 0 ), '' ), '\', '\\' ), '"', '\"' );
                for line = lines'
                    outputstring = append( outputstring, newline, "  ", '"', line, '"' );
                end
            end
            outputstring = append( outputstring, ";", newline );
        end
    end
    if standalone
        clc;
        disp( outputstring )
    else
        output = outputstring;
    end
end