/*
Copyright (c) 2010, Dan Bethell, Johannes Saam.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of RmanConnect nor the names of its contributors may be
    used to endorse or promote products derived from this software without
    specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RMAN_CONNECT_DATA_H_
#define RMAN_CONNECT_DATA_H_

#include <vector>
#include <boost/shared_array.hpp>

//! \namespace rmanconnect
namespace rmanconnect
{
    /*! \class Data
     * \brief Represents image information passed from Client to Server
     *
     * This class wraps up the data sent from Client to Server. When calling
     * Client::openImage() a Data object should first be constructed that
     * specifies the full image dimensions.
     * E.g. Data( 0, 0, 320, 240, 3 );
     *
     * When sending actually pixel information it should be constructed using
     * values that represent the chunk of pixels being sent.
     * E.g. Data( 15, 15, 16, 16, 3, myPixelPointer );
     */
    class Data
    {
    friend class Client;
    friend class Server;
    public:
        //! Constructor
        Data( int x, int y,
              int width, int height,
              int spp, std::string name,
              boost::shared_array<float> data );
        Data();
        //! Destructor
        ~Data();

        /*! \brief The 'type' of message this Data represents
         *
         * 0: image open
         * 1: pixels
         * 2: image close
         * 3: update
         */
        const int type() const { return mType; }
        void setType( int type ) { mType = type; }

        void setDimension(unsigned int width, unsigned int height ) { mWidth=width; mHeight=height; }
        //! X position
        int x() const { return mX; }
        //! y position
        int y() const { return mY; }
        //! Width
        int width() const { return mWidth; }
        //! Height
        int height() const { return mHeight; }
        //! Samples-per-pixel, aka channel depth
        int spp() const { return mSpp; }
        //! Pointer to pixel data owned by the display driver (client-side)
        boost::shared_array<float> data() const { return mpData; }
        //! Pointer to pixel data owned by this object (server-side)
        const float *pixels() const { return &mPixelStore[0]; }

        std::string& getName() { return mName; }

        Data& Data::operator=(const Data &input) {
            this->mX = input.mX;
            this->mY = input.mY;
            this->mType = input.mType;
            this->mWidth = input.mWidth;
            this->mName = input.mName;
            this->mHeight =input.mHeight;
            this->mSpp=input.mSpp;
            this->mpData=input.mpData;
            this->mPixelStore = input.mPixelStore;

            return *this;  // Return a reference to myself.
        }

    private:
        // what type of data is this?
        int mType;

        // x & y position
        int mX, mY;

        // name of the channel
        std::string mName;

        // width, height, num channels (samples)
        unsigned int mWidth, mHeight, mSpp;

        // our pixel data pointer (for driver-owned pixels)
        boost::shared_array<float> mpData;

        // our persistent pixel storage (for Data-owned pixels)
        std::vector<float> mPixelStore;
    };
}

#endif // RMAN_CONNECT_DATA_H_
