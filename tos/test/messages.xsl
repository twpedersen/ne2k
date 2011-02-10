<?xml version="1.0" encoding="utf-8"?>

<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
                version="1.0">

  <xsl:output method="html"/>

  <xsl:template match="/">
    <html>
      <body bgcolor="#FFFFFF">
        <table cellpadding="0" cellspacing="0" border="0" width="100%">
          <tr>
            <td bgcolor="#a0a0a0"><font color="#a0a0a0">X</font></td>
          </tr>
          <tr>
            <td bgcolor="#a0a0a0">
              <font color="#a0a0a0">XXXXXXXXX</font>
              <b><font size="+6">TOS Error Codes</font></b>
            </td>
          </tr>
          <tr>
            <td bgcolor="#a0a0a0"><font color="#a0a0a0">X</font></td>
          </tr>
        </table>
        <p/>
        <xsl:value-of select="TOS_error_codes/note"/>
        <p/>
        <xsl:for-each select="TOS_error_codes/error_code">
          <xsl:call-template name="emitAnchor"/>
        </xsl:for-each>
        <xsl:for-each select="TOS_error_codes/error_code">
          <xsl:call-template name="emitDescription"/>
        </xsl:for-each>
      </body>
    </html>
  </xsl:template>

  <xsl:template name="emitAnchor">
    <b><a href="#{@id}">Error code: <xsl:value-of select="@id"/></a></b>
    <br/>
  </xsl:template>

  <xsl:template name="emitDescription">
    <a name="{@id}"/>
    <p/><font color="#FFFFFF">.</font><p/>
    <table cellpadding="0" cellspacing="0" border="0" width="100%">
      <tr>
        <td bgcolor="#a0a0a0"><font color="#a0a0a0">XXXXX</font>
        <b>E<font size="-1">RROR</font>
        C<font size="-1">ODE</font>
        <font color="#a0a0a0">x</font>
        <xsl:value-of select="@id"/></b></td>
      </tr>
    </table>
    <p/>
    <table border="0">
      <tr>
        <td valign="top">
          <b>Description:</b>
        </td>
        <td valign="top">
          <xsl:value-of select="description"/>
        </td>
      </tr>
      <xsl:for-each select="possible_error_source">
        <tr>
          <td valign="top">
            <xsl:if test="position() = 1">
              <nobr><b>Possible source:</b></nobr>
            </xsl:if>
          </td>
          <td valign="top">
            <tt><xsl:value-of select="."/></tt>
          </td>
        </tr>
      </xsl:for-each>
    </table>
    <b>Hints:</b><br/>
    <ul>
      <xsl:for-each select="hints/hint">
        <li><xsl:value-of select="."/></li>
      </xsl:for-each>
    </ul>
    <p/>
  </xsl:template>

</xsl:stylesheet>
