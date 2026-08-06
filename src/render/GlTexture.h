#pragma once

// RAII owner of one GL texture name (the vendored toolkit has no texture
// wrapper). Deletes the texture on destruction, which runs while the GL
// context is live -- its holders sit inside Renderer (see Renderer's
// ownership note).
class GlTexture {
public:
    GlTexture() = default;                              // empty: no texture
    explicit GlTexture(unsigned int id) : m_id(id) {}   // adopts the name

    // Owns a raw GL id -- copying would double-free it.
    GlTexture(const GlTexture &) = delete;
    GlTexture &operator=(const GlTexture &) = delete;
    GlTexture(GlTexture &&other) noexcept;
    GlTexture &operator=(GlTexture &&other) noexcept;
    ~GlTexture();

    unsigned int id() const { return m_id; }   // 0 = none

private:
    unsigned int m_id = 0;
};
