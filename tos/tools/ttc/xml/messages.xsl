<?xml version="1.0" encoding="utf-8"?>

<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
                version="1.0">

  <xsl:param name="errno"/>

  <xsl:output method="html"/>

  <xsl:template match="/">
    <html>
      <body bgcolor="#FFFFFF">
        <xsl:for-each select="TOS_error_codes/error_code">
			<xsl:if test="@id = $errno">
                <xsl:call-template name="emitDescription"/>
			</xsl:if>
        </xsl:for-each>
      </body>
    </html>
  </xsl:template>

  <xsl:template name="emitDescription">
    <table border="0">
      <tr>
        <td valign="top">
          <b>Error code:</b>
        </td>
        <td valign="top">
          <xsl:value-of select="$errno"/>
        </td>
      </tr>
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
              <b><nobr>Possible source:</nobr></b>
            </xsl:if>
          </td>
          <td valign="top">
            <tt><xsl:value-of select="."/></tt>
          </td>
        </tr>
      </xsl:for-each>
	  <tr>
        <td valign="top"><b>Hints:</b></td>
		<td>
    <ul>
      <xsl:for-each select="hints/hint">
        <li><xsl:value-of select="."/></li>
      </xsl:for-each>
    </ul>
	</td>
	</tr>
    </table>
  </xsl:template>

</xsl:stylesheet>
