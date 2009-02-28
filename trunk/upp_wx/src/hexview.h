/***************************************************************************
*   Copyright (C) 2008 by Frans Schreuder                                 *
*   usbpicprog.sourceforge.net                                            *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#ifndef HEXVIEW_H
#define HEXVIEW_H

#include <wx/grid.h>

#include "hexfile.h"
#include "pictype.h"

#include <iostream>
using namespace std;


enum UppHexViewType
{
    HEXVIEW_CODE,
    HEXVIEW_CONFIG,
    HEXVIEW_DATA
};

class UppHexViewGrid : public wxGrid
{
public:
    UppHexViewGrid(wxWindow* parent, wxWindowID id,
                   UppHexViewType type);
    ~UppHexViewGrid();

    void Copy();
    void ShowHexFile(HexFile* hexFile, PicType* picType);

private:
    HexFile* m_hexFile;
    UppHexViewType m_type;

protected:
    void OnCopy (wxCommandEvent& event);
    void OnSelectAll (wxCommandEvent& event);
    void OnCellRightClicked (wxGridEvent& event );
    void OnCellChanged( wxGridEvent& event );

    wxSize DoGetBestSize() const;
};

#endif //HEXVIEW_H
