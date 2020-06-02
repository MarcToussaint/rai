
//////////////////////////////////////////////////////////////////////////////
//
// io.C
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <iomanip>
using std::cerr;
using std::ios;
using std::setprecision;

#include "../SWIFT/SWIFT.h"
#include "../SWIFT/SWIFT_mesh.h"
#include "../SWIFT/SWIFT_fileio.h"

typedef long int intptr_t;

extern bool machine_is_big_endian;
bool hierarchy = false;

bool Load_File( const char* filename, SWIFT_Tri_Mesh*& mesh,
                SPLIT_TYPE split, bool& already_decomp, bool& already_hier,
                SWIFT_Array<int>& piece_ids,
                SWIFT_Array< SWIFT_Array<int> >& mfs,
                SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs )
{
    SWIFT_FILE_TYPE ft;

    // Create the mesh
    if( mesh != nullptr ) {
        delete mesh;
    }

    if( !SWIFT_File_Type( filename, ft ) ) {
        return false;
    }

    switch( ft ) {
    case FT_CHIER:
        already_decomp = true;
        already_hier = true;
        // Turn on the hierarchy
        hierarchy = true;
        return SWIFT_File_Read_Hierarchy( filename, mesh );
        break;
    case FT_DECOMP:
        already_decomp = true;
        already_hier = false;
        return SWIFT_File_Read_Decomposition(
                        filename, mesh, false, piece_ids, mfs, vfs,
                        DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 1.0, split );
        break;
    case FT_OTHER:
      { SWIFT_Real* vs = nullptr;
        int* fs = nullptr;
        int vn, fn;
        int* fv = nullptr;

        if( !global_scene->get_File_Read_Dispatcher()->Read( filename, vs, fs, vn, fn, fv ) ) {
            // Delete everything
            delete vs; delete fs; delete fv;

            cerr << "Error: \"" << filename << "\" file read failed" << endl;
            return false;
        }

        mesh = new SWIFT_Tri_Mesh;

        if( !mesh->Create( vs, fs, vn, fn, DEFAULT_ORIENTATION,
                           DEFAULT_TRANSLATION, 1.0, fv )
        ) {
            // Delete everything
            delete mesh; delete vs; delete fs; delete fv;

            cerr << "Error: \"" << filename
                 << "\" mesh creation failed" << endl;
            return false;
        }

        delete vs; delete fs; delete fv;
        already_decomp = false;
        already_hier = false;
        return true;
      }
        break;
    default:
        cerr << "Error: Unrecognized type of file from File_IO package" << endl;
        return false;
        break;
    }
}

// Save a normal model file
bool Save_Model_File( const char* filename, SWIFT_Tri_Mesh* m )
{
    int i;
    ofstream fout;

    // Try to open the file
    if( filename == nullptr ) {
        cerr << "Error: Invalid filename given to write model" << endl;
        return false;
    }

    fout.open( filename, ios::out );

    if( !fout.rdbuf()->is_open( ) ) {
        cerr << "Error: file could not be opened for writing \""
             << filename << "\"" << endl;
        return false;
    }

    fout << setprecision( 16 );
    fout << "TRI" << endl << endl << m->Num_Vertices() << endl << endl
         << m->Num_Faces() << endl << endl;

    for( i = 0; i < m->Num_Vertices(); i++ ) {
        fout << m->Vertices()[i].Coords().X() << " "
             << m->Vertices()[i].Coords().Y() << " "
             << m->Vertices()[i].Coords().Z() << endl;
    }
    fout << endl;

    for( i = 0; i < m->Num_Faces(); i++ ) {
        fout << m->Vertex_Id( m->Faces()[i].Edge1().Origin() ) << " "
             << m->Vertex_Id( m->Faces()[i].Edge2().Origin() ) << " "
             << m->Vertex_Id( m->Faces()[i].Edge3().Origin() ) << endl;
    }
    fout << endl;

    fout.close();

    return true;
}

// Save a decomposition file
bool Save_Decomposition_File( const char* filename, SWIFT_Tri_Mesh* m,
                              SWIFT_Array<int>& piece_ids,
                              SWIFT_Array< SWIFT_Array<int> >& mfs,
                              SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs )
{
    int i, j, k;
    int num_vfaces;
    int num_mfaces;
    ofstream fout;
    SWIFT_Real* vs;
    SWIFT_Real* vs_walk;
    int* fs;

    // Try to open the file
    if( filename == nullptr ) {
        cerr << "Error: Invalid filename given to write decomp" << endl;
        return false;
    }

#ifdef WIN32
    fout.open( filename, ios::out | ios::binary );
#else
    fout.open( filename, ios::out );
#endif

    if( !fout.rdbuf()->is_open( ) ) {
        cerr << "Error: file could not be opened for writing \""
             << filename << "\"" << endl;
        return false;
    }

    fout << '\0';
    fout << "Convex_Decomposition" << endl;

    if( machine_is_big_endian ) {
        fout << "binary big_endian" << endl;
    } else {
        fout << "binary little_endian" << endl;
    }

    if( sizeof(SWIFT_Real) == sizeof(float) ) {
        fout << "real float" << endl;
    } else {
        fout << "real double" << endl;
    }

    fout << "vertices " << m->Num_Vertices() << endl;
    fout << "faces " << m->Num_Faces() << endl;
    fout << "map_vids " << m->Map_Vertex_Ids().Length() << endl;
    fout << "map_fids " << m->Map_Face_Ids().Length() << endl;
    fout << "pieces " << mfs.Length() << endl;

    // Compose the vertices into an array
    vs = new SWIFT_Real[m->Num_Vertices()*3];
    vs_walk = vs;
    for( i = 0; i < m->Num_Vertices(); i++, vs_walk += 3 ) {
        m->Vertices()[i].Coords().Get_Value( vs_walk );
    }

    // Write the vertices
    fout.write( (char*)vs, m->Num_Vertices()*3*sizeof(SWIFT_Real) );
    delete [] vs;

    // Compose the faces into an array
    fs = new int[m->Num_Faces()*3];
    for( i = 0, j = 0; i < m->Num_Faces(); i++, j += 3 ) {
        fs[j] = m->Vertex_Id( m->Faces()[i].Edge1().Origin() );
        fs[j+1] = m->Vertex_Id( m->Faces()[i].Edge2().Origin() );
        fs[j+2] = m->Vertex_Id( m->Faces()[i].Edge3().Origin() );
    }

    // Write out the faces
    fout.write( (char*)fs, m->Num_Faces()*3*sizeof(int) );
    delete [] fs;

    // Write out vertex mapping
    if( !m->No_Duplicate_Vertices() ) {
        fout.write( (char*)m->Map_Vertex_Ids().Data(),
                    m->Num_Vertices()*sizeof(int) );
    }

    // Write out face mapping
    if( !m->Only_Triangles() ) {
        fout.write( (char*)m->Map_Face_Ids().Data(),
                    m->Num_Faces()*sizeof(int) );
    }

    // Write out piece ids array
    fout.write( (char*)piece_ids.Data(), piece_ids.Length()*sizeof(int) );

    // Compose the original faces lengths into an array
    fs = new int[mfs.Length()];
    for( i = 0; i < mfs.Length(); i++ ) {
        fs[i] = mfs[i].Length();
    }

    // Write out the original faces lengths
    fout.write( (char*)fs, mfs.Length()*sizeof(int) );
    delete [] fs;

    // Compose the virtual faces lengths into an array
    num_vfaces = 0;
    fs = new int[vfs.Length()];
    for( i = 0; i < vfs.Length(); i++ ) {
        num_vfaces += vfs[i].Length();
        fs[i] = vfs[i].Length();
    }

    // Write out the virtual faces lengths
    fout.write( (char*)fs, vfs.Length()*sizeof(int) );
    delete [] fs;

    // Compose the original face ids into an array
    num_mfaces = 0;
    for( i = 0; i < mfs.Length(); i++ ) {
        num_mfaces += mfs[i].Length();
    }
    fs = new int[num_mfaces];
    for( i = 0, k = 0; i < mfs.Length(); i++ ) {
        for( j = 0; j < mfs[i].Length(); j++, k++ ) {
            fs[k] = mfs[i][j];
        }
    }

    // Write out the original face ids
    fout.write( (char*)fs, num_mfaces*sizeof(int) );
    delete [] fs;

    // Compose the virtual face vertex ids into an array
    fs = new int[num_vfaces*3];
    for( i = 0, k = 0; i < vfs.Length(); i++ ) {
        for( j = 0; j < vfs[i].Length(); j++, k += 3 ) {
            fs[k] = m->Vertex_Id( vfs[i][j].Edge1().Origin() );
            fs[k+1] = m->Vertex_Id( vfs[i][j].Edge2().Origin() );
            fs[k+2] = m->Vertex_Id( vfs[i][j].Edge3().Origin() );
        }
    }

    // Write out the virtual face vertex ids
    fout.write( (char*)fs, num_vfaces*3*sizeof(int) );
    delete [] fs;

    fout.close();

    return true;
}

// Save a hierarchy file
bool Save_Hierarchy_File( const char* filename, SWIFT_Tri_Mesh* m,
                          SWIFT_Array<SWIFT_Tri_Face*>& st_faces,    
                          SWIFT_Array<SWIFT_Tri_Edge*>& st_twins )
{
    ofstream fout;

    // Try to open the file
    if( filename == nullptr ) {
        cerr << "Error: Invalid filename given to write model" << endl;
        return false;
    }

#ifdef WIN32
    fout.open( filename, ios::out | ios::binary );
#else
    fout.open( filename, ios::out );
#endif

    if( !fout.rdbuf()->is_open( ) ) {
        cerr << "Error: file could not be opened for writing \""
             << filename << "\"" << endl;
        return false;
    }

    int i, j, k, l;
    int ntf, ncf, net;
    SWIFT_Array<int> bvq( m->Num_BVs() );
    SWIFT_Array<int> bvs_bmap( m->Num_BVs() );
    SWIFT_Array<int> flen_accum( m->Num_BVs()+1 );
    SWIFT_Array<int> cflen_accum( m->Num_BVs()+1 );
    SWIFT_Tri_Edge* e;
    SWIFT_Tri_Face* f;
    SWIFT_Real* vs;
    SWIFT_Real* vs_walk;
    int* fs;
    char* cs;

    fout << '\0';
    fout << "Convex_Hierarchy" << endl;

    if( machine_is_big_endian ) {
        fout << "binary big_endian" << endl;
    } else {
        fout << "binary little_endian" << endl;
    }

    if( sizeof(SWIFT_Real) == sizeof(float) ) {
        fout << "real float" << endl;
    } else {
        fout << "real double" << endl;
    }

    // Traverse the hierarchy BF counting stuff
    bvq.Set_Length( 0 );
    bvq.Add( 0 );
    bvs_bmap[0] = 0;

    ntf = m->Num_Faces();
    ncf = 0;
    net = 0;

    // Count total twin lengths
    for( i = 0; i < m->Num_Faces(); i++ ) {
        net += m->Faces()[i].Twins_Length();
    }
    
    flen_accum[0] = ntf;
    cflen_accum[0] = ncf;

    for( i = 0; i < bvq.Length(); i++ ) {
        for( j = 0; j < m->BVs()[bvq[i]].Num_Faces(); j++ ) {
            net += m->BVs()[bvq[i]].Faces()[j].Twins_Length();
        }
        ntf += m->BVs()[bvq[i]].Num_Faces();
        ncf += m->BVs()[bvq[i]].Num_Other_Faces();
        flen_accum[i+1] = ntf;
        cflen_accum[i+1] = ncf;
        if( m->BVs()[bvq[i]].Num_Children() != 0 ) {
            bvq.Add( m->BVs().Position( m->BVs()[bvq[i]].Children()[0] ) );
            bvs_bmap[bvq.Last()] = bvq.Length()-1;
            bvq.Add( m->BVs().Position( m->BVs()[bvq[i]].Children()[1] ) );
            bvs_bmap[bvq.Last()] = bvq.Length()-1;
        }
    }

    net *= 3;

    // Write out the counts
    fout << "vertices " << m->Num_Vertices() << endl;
    fout << "map_vids " << m->Map_Vertex_Ids().Length() << endl;
    fout << "map_fids " << m->Map_Face_Ids().Length() << endl;
    fout << "orig_faces " << m->Num_Faces() << endl;
    fout << "orig_hier_faces " << st_faces.Length() << endl;
    fout << "total_faces " << ntf << endl;
    fout << "copied_faces " << ncf << endl;
    fout << "edge_twins " << net << endl;
    fout << "nodes " << m->Num_BVs() << endl;

    // Write out the SWIFT_Tri_Mesh record
    cs = new char[sizeof(int)+7*sizeof(SWIFT_Real)];

    *((int*)cs) = m->Height();
    *((SWIFT_Real*)(cs+sizeof(int))) = m->Center_Of_Mass().X();
    *((SWIFT_Real*)(cs+sizeof(int)+sizeof(SWIFT_Real))) =
                                                m->Center_Of_Mass().Y();
    *((SWIFT_Real*)(cs+sizeof(int)+2*sizeof(SWIFT_Real))) =
                                                m->Center_Of_Mass().Z();
    *((SWIFT_Real*)(cs+sizeof(int)+3*sizeof(SWIFT_Real))) = m->Center().X();
    *((SWIFT_Real*)(cs+sizeof(int)+4*sizeof(SWIFT_Real))) = m->Center().Y();
    *((SWIFT_Real*)(cs+sizeof(int)+5*sizeof(SWIFT_Real))) = m->Center().Z();
    *((SWIFT_Real*)(cs+sizeof(int)+6*sizeof(SWIFT_Real))) = m->Radius();

    fout.write( cs, sizeof(int)+7*sizeof(SWIFT_Real) );

    delete cs;

    // Compose the vertices into an array
    vs = new SWIFT_Real[m->Num_Vertices()*3];
    vs_walk = vs;
    for( i = 0; i < m->Num_Vertices(); i++, vs_walk += 3 ) {
        m->Vertices()[i].Coords().Get_Value( vs_walk );
    }

    // Write the vertices
    fout.write( (char*)vs, m->Num_Vertices()*3*sizeof(SWIFT_Real) );
    delete vs;

    // Write out vertex mapping
    if( !m->No_Duplicate_Vertices() ) {
        fout.write( (char*)m->Map_Vertex_Ids().Data(),
                    m->Num_Vertices()*sizeof(int) );
    }

    // Write out face mapping
    if( !m->Only_Triangles() ) {
        fout.write( (char*)m->Map_Face_Ids().Data(),
                    m->Num_Faces()*sizeof(int) );
    }

    // Write out the faces
    fs = new int[ntf*4];

    // faces in the mesh
    for( i = 0, k = 0, l = 0; i < m->Num_Faces(); i++, k += 4, l++ ) {
        fs[k] = m->Vertex_Id( m->Faces()[i].Vertex1() );
        fs[k+1] = m->Vertex_Id( m->Faces()[i].Vertex2() );
        fs[k+2] = m->Vertex_Id( m->Faces()[i].Vertex3() );
        fs[k+3] = m->Faces()[i].Bit_Field();
        // Store the global id of this face in the face
        m->Faces()[i].Edge1().Set_Next( (SWIFT_Tri_Edge*)l );
    }

    // faces in the hierarchy
    for( i = 0; i < bvq.Length(); i++ ) {
        for( j = 0; j < m->BVs()[bvq[i]].Num_Faces(); j++, k += 4, l++ ){
            fs[k] = m->Vertex_Id( m->BVs()[bvq[i]].Faces()[j].Vertex1() );
            fs[k+1] = m->Vertex_Id( m->BVs()[bvq[i]].Faces()[j].Vertex2() );
            fs[k+2] = m->Vertex_Id( m->BVs()[bvq[i]].Faces()[j].Vertex3() );
            fs[k+3] = m->BVs()[bvq[i]].Faces()[j].Bit_Field();
            // Store the global id of this face in the face
            m->BVs()[bvq[i]].Faces()[j].Edge1().Set_Next(
                                                    (SWIFT_Tri_Edge*)l );
        }
    }

    fout.write( (char*)fs, ntf*4*sizeof(int) );
    delete fs;

    // Write the copied face indices
    fs = new int[ncf];
    for( i = 0, k = 0; i < bvq.Length(); i++ ) {
        for( j = 0; j < m->BVs()[bvq[i]].Num_Other_Faces(); j++, k++ ) {
            // Global id is simply stored in the face
            fs[k] = ( intptr_t)(m->BVs()[bvq[i]].Other_Faces()[j]->
                                                        Edge1().Next());
        }
    }

    fout.write( (char*)fs, ncf*sizeof(int) );
    delete fs;

    // Write out original twins
    fs = new int[3*m->Num_Faces()];
    for( i = 0, l = 0; i < m->Num_Faces(); i++, l += 3 ) {
        e = m->Faces()[i].Edge1().Twin();
        fs[l] = (e == nullptr ? -1 :
                            (( intptr_t)(e->Adj_Face()->Edge1().Next())<<2) +
                             e->Adj_Face()->Edge_Id( e ));

        e = m->Faces()[i].Edge2().Twin();
        fs[l+1] = (e == nullptr ? -1 :
                            (( intptr_t)(e->Adj_Face()->Edge1().Next())<<2) +
                             e->Adj_Face()->Edge_Id( e ));

        e = m->Faces()[i].Edge3().Twin();
        fs[l+2] = (e == nullptr ? -1 :
                            (( intptr_t)(e->Adj_Face()->Edge1().Next())<<2) +
                             e->Adj_Face()->Edge_Id( e ));
    }

    fout.write( (char*)fs, 3*m->Num_Faces()*sizeof(int) );
    delete fs;

    // Write out ids of original faces living in the hierarchy along with their
    // twins which point to edges in the main mesh.
    fs = new int[st_faces.Length()+st_twins.Length()];
    for( i = 0, j = 0, k = 0; i < st_faces.Length(); i++, j += 3, k += 4 ) {
        fs[k] = ( intptr_t)(st_faces[i]->Edge1().Next());
        f = st_twins[j]->Adj_Face();
        fs[k+1] = (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( st_twins[j] );
        f = st_twins[j+1]->Adj_Face();
        fs[k+2] = (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( st_twins[j+1] );
        f = st_twins[j+2]->Adj_Face();
        fs[k+3] = (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( st_twins[j+2] );
    }

    fout.write( (char*)fs, (st_faces.Length()+st_twins.Length())*sizeof(int) );
    delete fs;

    // Write the edge twins
    fs = new int[net];
    for( i = 0, l = 0; i < m->Num_Faces();
         l += 3*m->Faces()[i].Twins_Length(), i++
    ) {
        const int first_base = l;
        const int second_base = l+m->Faces()[i].Twins_Length();
        const int third_base = l+(m->Faces()[i].Twins_Length()<<1);
        const int face_level = m->Faces()[i].Starting_Level();
        for( k = face_level;
             k < m->Faces()[i].Twins_Length() + face_level; k++
        ) {
            e = m->Faces()[i].Edge1().Twin( k );
            f = e->Adj_Face();
            fs[first_base+k-face_level] =
                            (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( e );

            e = m->Faces()[i].Edge2().Twin( k );
            f = e->Adj_Face();
            fs[second_base+k-face_level] =
                            (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( e );

            e = m->Faces()[i].Edge3().Twin( k );
            f = e->Adj_Face();
            fs[third_base+k-face_level] = 
                            (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( e );
        }
    }

    for( i = 0; i < bvq.Length(); i++ ) {
        for( j = 0; j < m->BVs()[bvq[i]].Num_Faces();
             l += 3*m->BVs()[bvq[i]].Faces()[j].Twins_Length(), j++
        ) {
            const int first_base = l;
            const int second_base =
                            l+m->BVs()[bvq[i]].Faces()[j].Twins_Length();
            const int third_base =
                        l+(m->BVs()[bvq[i]].Faces()[j].Twins_Length()<<1);
            const int face_level =
                        m->BVs()[bvq[i]].Faces()[j].Starting_Level();
            for( k = face_level; k < m->BVs()[bvq[i]].Faces()[j].
                                           Twins_Length() + face_level; k++
            ) {
                e = m->BVs()[bvq[i]].Faces()[j].Edge1().Twin( k );
                f = e->Adj_Face();
                fs[first_base+k-face_level] =
                            (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( e );

                e = m->BVs()[bvq[i]].Faces()[j].Edge2().Twin( k );
                f = e->Adj_Face();
                fs[second_base+k-face_level] =
                            (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( e );

                e = m->BVs()[bvq[i]].Faces()[j].Edge3().Twin( k );
                f = e->Adj_Face();
                fs[third_base+k-face_level] = 
                            (( intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( e );
            }
        }
    }

    fout.write( (char*)fs, net*sizeof(int) );
    delete fs;

    // Write the BV records
    for( i = 0; i < bvq.Length(); i++ ) {
        const int BV_bytes = 9*sizeof(int)+4*sizeof(SWIFT_Real)+
                     m->BVs()[bvq[i]].Lookup_Table_Size()*sizeof(int);
        int cs_off = 0;
        cs = new char[BV_bytes];

        // Copied faces
        *((int*)cs+cs_off) = m->BVs()[bvq[i]].Num_Other_Faces();
        cs_off += sizeof(int);
        *((int*)(cs+cs_off)) = cflen_accum[i];
        cs_off += sizeof(int);

        // Faces
        *((int*)(cs+cs_off)) = m->BVs()[bvq[i]].Num_Faces();
        cs_off += sizeof(int);
        *((int*)(cs+cs_off)) = flen_accum[i];
        cs_off += sizeof(int);

        // Parent
        *((int*)(cs+cs_off)) = (i == 0 ? -1 :
                    bvs_bmap[m->BVs().Position(m->BVs()[bvq[i]].Parent())]);
        cs_off += sizeof(int);

        // Children
        *((int*)(cs+cs_off)) = m->BVs()[bvq[i]].Num_Children() == 0 ? -1 :
            bvs_bmap[m->BVs().Position(m->BVs()[bvq[i]].Children()[0])];
        cs_off += sizeof(int);
        *((int*)(cs+cs_off)) = m->BVs()[bvq[i]].Num_Children() == 0 ? -1 :
            bvs_bmap[m->BVs().Position(m->BVs()[bvq[i]].Children()[1])];
        cs_off += sizeof(int);

        // Level
        *((int*)(cs+cs_off)) = m->BVs()[bvq[i]].Level();
        cs_off += sizeof(int);

        // COM, Radius
        *((SWIFT_Real*)(cs+cs_off)) = m->BVs()[bvq[i]].Center_Of_Mass().X();
        cs_off += sizeof(SWIFT_Real);
        *((SWIFT_Real*)(cs+cs_off)) = m->BVs()[bvq[i]].Center_Of_Mass().Y();
        cs_off += sizeof(SWIFT_Real);
        *((SWIFT_Real*)(cs+cs_off)) = m->BVs()[bvq[i]].Center_Of_Mass().Z();
        cs_off += sizeof(SWIFT_Real);
        *((SWIFT_Real*)(cs+cs_off)) = m->BVs()[bvq[i]].Radius();
        cs_off += sizeof(SWIFT_Real);

        // LUT
        *((int*)(cs+cs_off)) = m->BVs()[bvq[i]].Lookup_Table().Type();
        cs_off += sizeof(int);

        for( k = 0; k < m->BVs()[bvq[i]].Lookup_Table_Size(); k++ ) {
            e = m->BVs()[bvq[i]].Lookup_Table().Table()[k];
            f = e->Adj_Face();
            ((int*)(cs+cs_off))[k] =
                            ((intptr_t)(f->Edge1().Next())<<2) + f->Edge_Id( e );
        }

        fout.write( cs, BV_bytes );
        delete cs;
    }

    // Restore the edge1 next ptrs
    for( i = 0; i < m->Num_Faces(); i++ ) {
        m->Faces()[i].Edge1().Set_Next( m->Faces()[i].Edge2P() );
    }
    for( i = 0; i < bvq.Length(); i++ ) {
        for( j = 0; j < m->BVs()[bvq[i]].Num_Faces(); j++ ) {
            m->BVs()[bvq[i]].Faces()[j].Edge1().Set_Next(
                                m->BVs()[bvq[i]].Faces()[j].Edge2P() );
        }
    }

    fout.close();

    return true;
}



